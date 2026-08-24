#!/usr/bin/env python3
"""`.vscode/` 一式(settings/tasks/launch/extensions)を自動生成する。

  python tools/gen_vscode.py

ハードコードされがちな pico-sdk の各コンポーネント版数(sdk/toolchain/openocd/
picotool/cmake/ninja)を ~/.pico-sdk から自動検出して埋め込む。SDK を更新しても
このスクリプトを再実行すれば .vscode が追従する(手編集は不要)。

生成物:
  settings.json   env 注入 + CMake 設定 + ステータスバー Build/Deploy/OTA ボタン
  tasks.json      Compile / Deploy(USB) / Flash(SWD) / Bazel / OTA / GDB橋 / 再生成
  launch.json     Pico Debug   = ビルド→openocd→load→main で停止 (要 SWD プローブ)
                  GDB over BLE = 橋を起こして attach (プローブ不要・無線だけ)
  extensions.json 推奨拡張

★ビルド系が 2 本ある。CMake (build/main.elf) が従来の経路で、Bazel
  (bazel-bin/firmware_bazel/xno_bringup) が OTA と GDB-over-BLE が使う経路。
  混ぜないこと —— OTA で送る像と GDB に食わせるシンボルは同じ物でないと、
  「行番号だけずれた嘘のデバッグ」になる。

c_cpp_properties.json / cmake-kits.json は管理対象外(そのまま残す)。

将来 esp32 / host-sim / x64-uefi を足すときは TARGETS に定義を追加し、_launch()/
_tasks() を arch で分岐させる。今は rp2350 (pico2_w) のみ。
"""

import glob
import json
import os
import shutil
import sys

HOME = os.path.expanduser("~")
PICO = os.path.join(HOME, ".pico-sdk")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VSCODE = os.path.join(ROOT, ".vscode")


def detect(sub: str) -> str:
    """~/.pico-sdk/<sub>/ 直下の版数ディレクトリ名を返す(複数あれば最新)。"""
    paths = sorted(glob.glob(os.path.join(PICO, sub, "*")))
    paths = [p for p in paths if os.path.isdir(p)]
    if not paths:
        sys.exit(f"[gen_vscode] pico-sdk コンポーネントが見つかりません: {sub}")
    return os.path.basename(paths[-1])


# ---- 版数の自動検出 --------------------------------------------------------
SDK = detect("sdk")            # 例 2.2.0
TOOLCHAIN = detect("toolchain")  # 例 14_2_Rel1
OPENOCD = detect("openocd")    # 例 0.12.0+dev
PICOTOOL = detect("picotool")  # 例 2.2.0-a4
CMAKE = detect("cmake")        # 例 v3.31.5
NINJA = detect("ninja")        # 例 v1.12.1

# ---- ターゲット定義 (将来 esp/host/uefi を足す起点) ------------------------
BOARD = "pico2_w"
CHIP = "rp2350"
CHIP_UP = "RP2350"

# VS Code / タスクが使う Python インタプリタ。
PYTHON = "/opt/homebrew/bin/python3"
BAZEL = shutil.which("bazel") or "/opt/homebrew/bin/bazel"

# ~ を VS Code 変数で。版数は生成時に確定させる。
P = "${userHome}/.pico-sdk"
OPENOCD_BIN = f"{P}/openocd/{OPENOCD}/openocd"
OPENOCD_SCRIPTS = f"{P}/openocd/{OPENOCD}/scripts"
PICOTOOL_BIN = f"{P}/picotool/{PICOTOOL}/picotool/picotool"
CMAKE_BIN = f"{P}/cmake/{CMAKE}/bin/cmake"
NINJA_BIN = f"{P}/ninja/{NINJA}/ninja"
GDB_BIN = f"{P}/toolchain/{TOOLCHAIN}/bin/arm-none-eabi-gdb"
SVD = f"{P}/sdk/{SDK}/src/{CHIP}/hardware_regs/{CHIP_UP}.svd"
ELF = "${workspaceFolder}/build/main.elf"
UF2 = "${workspaceFolder}/build/main.uf2"

# ---- Bazel 側 (OTA / GDB-over-BLE が使う) ----------------------------------
BAZEL_TARGET = "//firmware_bazel:xno_bringup"
BAZEL_ELF = "${workspaceFolder}/bazel-bin/firmware_bazel/xno_bringup"
# ★ソースを引くのに要る。bazel の DWARF は execroot からの相対パス
#   (`external/shizuku+/...`, `firmware_bazel/main.cpp`) なので、gdb に
#   execroot を教えないと「No such file or directory」になる。
BAZEL_EXECROOT = "${workspaceFolder}/bazel-" + os.path.basename(ROOT)
OTA_SEND = "${workspaceFolder}/tools/ota_send.py"
GDB_BRIDGE = "${workspaceFolder}/tools/gdb_ble_bridge.py"
GDB_BLE_PORT = 3333


def _env(home_var: str, sep: str) -> dict:
    root = f"${{env:{home_var}}}/.pico-sdk"
    path = sep.join([
        f"{root}/toolchain/{TOOLCHAIN}/bin",
        f"{root}/picotool/{PICOTOOL}/picotool",
        f"{root}/cmake/{CMAKE}/bin",
        f"{root}/ninja/{NINJA}",
        f"${{env:{'Path' if home_var == 'USERPROFILE' else 'PATH'}}}",
    ])
    return {
        "PICO_SDK_PATH": f"{root}/sdk/{SDK}",
        "PICO_TOOLCHAIN_PATH": f"{root}/toolchain/{TOOLCHAIN}",
        ("Path" if home_var == "USERPROFILE" else "PATH"): path,
    }


def settings() -> dict:
    return {
        "cmake.showSystemKits": False,
        "cmake.options.statusBarVisibility": "hidden",
        "cmake.options.advanced": {
            "build": {"statusBarVisibility": "hidden"},
            "launch": {"statusBarVisibility": "hidden"},
            "debug": {"statusBarVisibility": "hidden"},
            "variant": {"statusBarVisibility": "compact"},
            "buildTarget": {"statusBarVisibility": "visible"},
        },
        "cmake.configureOnEdit": True,
        "cmake.automaticReconfigure": True,
        "cmake.configureOnOpen": False,
        "cmake.generator": "Ninja",
        "cmake.cmakePath": CMAKE_BIN,
        "C_Cpp.debugShortcut": False,
        "terminal.integrated.env.windows": _env("USERPROFILE", ";"),
        "terminal.integrated.env.osx": _env("HOME", ":"),
        "terminal.integrated.env.linux": _env("HOME", ":"),
        "raspberry-pi-pico.cmakeAutoConfigure": False,
        "raspberry-pi-pico.useCmakeTools": True,
        "raspberry-pi-pico.cmakePath": CMAKE_BIN,
        "raspberry-pi-pico.ninjaPath": NINJA_BIN,
        "python-envs.defaultEnvManager": "ms-python.python:system",
        "python.defaultInterpreterPath": PYTHON,
        "actionButtons": {
            "defaultColor": "#a3be8c",
            "loadNpmCommands": False,
            "reloadButton": None,
            "commands": [
                {
                    "name": "$(tools) Build",
                    "tooltip": "cmake --build build",
                    "color": "#a3be8c",
                    "singleInstance": True,
                    "command": "cmake --build build",
                },
                {
                    "name": "$(cloud-upload) Deploy",
                    "tooltip": "USB(BOOTSEL) で書き込み→実行 (picotool -fx)",
                    "color": "#88c0d0",
                    "command": f"picotool load {UF2} -fx",
                },
                {
                    # ★USB を挿さずに焼き替える。転送 50 秒 + commit 6.5 秒。
                    "name": "$(radio-tower) OTA",
                    "tooltip": "BLE だけで焼き替える (bazel build → 転送 → commit)",
                    "color": "#ebcb8b",
                    "singleInstance": True,
                    "command": (f"{BAZEL} build {BAZEL_TARGET} && "
                                f"{PYTHON} -u {OTA_SEND} --commit"),
                },
            ],
        },
    }


def tasks() -> dict:
    return {
        "version": "2.0.0",
        "tasks": [
            {
                "label": "Compile Project",
                "type": "process",
                "isBuildCommand": True,
                "command": CMAKE_BIN,
                "args": ["--build", "${workspaceFolder}/build"],
                "group": {"kind": "build", "isDefault": True},
                "presentation": {"reveal": "always", "panel": "dedicated"},
                "problemMatcher": "$gcc",
            },
            {
                "label": "Deploy (USB)",
                "type": "process",
                "dependsOn": ["Compile Project"],
                "dependsOrder": "sequence",
                "command": PICOTOOL_BIN,
                "args": ["load", UF2, "-fx"],
                "presentation": {"reveal": "always", "panel": "dedicated"},
                "problemMatcher": [],
            },
            {
                "label": "Flash (SWD)",
                "type": "process",
                "dependsOn": ["Compile Project"],
                "dependsOrder": "sequence",
                "command": OPENOCD_BIN,
                "args": [
                    "-s", OPENOCD_SCRIPTS,
                    "-f", "interface/cmsis-dap.cfg",
                    "-f", f"target/{CHIP}.cfg",
                    "-c", f'adapter speed 5000; program "{ELF}" verify reset exit',
                ],
                "problemMatcher": [],
            },
            {
                "label": "Build (Bazel)",
                "type": "process",
                "command": BAZEL,
                "args": ["build", BAZEL_TARGET],
                "options": {"cwd": "${workspaceFolder}"},
                "group": "build",
                "presentation": {"reveal": "always", "panel": "dedicated"},
                "problemMatcher": "$gcc",
            },
            {
                # ★OTA のワンライナー。ota_send.py が .elf をそのまま食えるので、
                #   .bin へ落とす手順は要らない (中身を見て自分で変換する)。
                "label": "OTA Deploy (BLE)",
                "type": "process",
                "dependsOn": ["Build (Bazel)"],
                "dependsOrder": "sequence",
                "command": PYTHON,
                "args": ["-u", OTA_SEND, "--commit"],
                "options": {"cwd": "${workspaceFolder}"},
                "presentation": {"reveal": "always", "panel": "dedicated"},
                "problemMatcher": [],
            },
            {
                # 本体には触れず、ステージングへ置くだけ (commit は後から
                # `--commit-only` でよい — デバイスは flash から読み直して
                # 検証するので、別の接続でかまわない)。
                "label": "OTA Stage only (BLE)",
                "type": "process",
                "dependsOn": ["Build (Bazel)"],
                "dependsOrder": "sequence",
                "command": PYTHON,
                "args": ["-u", OTA_SEND],
                "options": {"cwd": "${workspaceFolder}"},
                "presentation": {"reveal": "always", "panel": "dedicated"},
                "problemMatcher": [],
            },
            {
                # ★launch の "GDB over BLE" が preLaunchTask に使う常駐タスク。
                #   isBackground なので、endsPattern の行が出た時点で
                #   VS Code はデバッガの起動へ進む (スキャンに 15 秒かかるため、
                #   「待つ」ことを明示しないと attach が先走って失敗する)。
                "label": "GDB BLE bridge",
                "type": "process",
                "command": PYTHON,
                "args": ["-u", GDB_BRIDGE, "--port", str(GDB_BLE_PORT)],
                "options": {"cwd": "${workspaceFolder}"},
                "isBackground": True,
                "presentation": {"reveal": "always", "panel": "dedicated"},
                "problemMatcher": {
                    "pattern": {"regexp": "^$", "file": 1, "location": 2,
                                "message": 3},
                    "background": {
                        "activeOnStart": True,
                        "beginsPattern": "^found: ",
                        "endsPattern": "^listening on ",
                    },
                },
            },
            {
                "label": "Regenerate .vscode",
                "type": "process",
                "command": PYTHON,
                "args": ["${workspaceFolder}/tools/gen_vscode.py"],
                "presentation": {"reveal": "always", "panel": "shared"},
                "problemMatcher": [],
            },
        ],
    }


def launch() -> dict:
    return {
        "version": "0.2.0",
        "configurations": [
            {
                "name": f"Pico Debug ({CHIP_UP})",
                "preLaunchTask": "Compile Project",
                "type": "cortex-debug",
                "request": "launch",
                "servertype": "openocd",
                "cwd": "${workspaceFolder}",
                "serverpath": OPENOCD_BIN,
                "gdbPath": GDB_BIN,
                "executable": ELF,
                "device": CHIP_UP,
                "configFiles": ["interface/cmsis-dap.cfg", f"target/{CHIP}.cfg"],
                "searchDir": [OPENOCD_SCRIPTS],
                "svdFile": SVD,
                "runToEntryPoint": "main",
                "openOCDLaunchCommands": ["adapter speed 5000"],
                "overrideLaunchCommands": [
                    "monitor reset init",
                    f'load "{ELF}"',
                ],
            },
            {
                # ★プローブ無しで止めて覗く。SWD の halting debug と違って
                #   DebugMonitor なので **止まるのは対象のスレッドだけ** ——
                #   BLE も走り続ける (走り続けないと RSP を運べない)。
                #   対象は main.cpp が stub に渡したスレッド (blink)。
                "name": "GDB over BLE (attach)",
                "preLaunchTask": "GDB BLE bridge",
                "type": "cortex-debug",
                "request": "attach",
                "servertype": "external",
                "gdbTarget": f"localhost:{GDB_BLE_PORT}",
                "cwd": "${workspaceFolder}",
                "gdbPath": GDB_BIN,
                "executable": BAZEL_ELF,
                "device": CHIP_UP,
                "svdFile": SVD,
                # ★`target extended-remote` を撃たせない。stub が喋るのは
                #   素の remote プロトコルだけなので、既定の attach 手順に
                #   任せず自分で並べる。
                "overrideAttachCommands": [
                    f'directory "{BAZEL_EXECROOT}"',
                    f"target remote localhost:{GDB_BLE_PORT}",
                ],
                "overrideRestartCommands": [],
            },
        ],
    }


def extensions() -> dict:
    return {
        "recommendations": [
            "marus25.cortex-debug",
            "ms-vscode.cpptools",
            "ms-vscode.cpptools-extension-pack",
            "ms-vscode.vscode-serial-monitor",
            "raspberry-pi.raspberry-pi-pico",
            "seunlanlege.action-buttons",
        ]
    }


def write(name: str, obj: dict):
    path = os.path.join(VSCODE, name)
    with open(path, "w", encoding="utf-8") as f:
        f.write("// AUTO-GENERATED by tools/gen_vscode.py — このファイルは手編集しない。\n")
        f.write("// 変更は gen_vscode.py 側で行い、再生成すること。\n")
        json.dump(obj, f, indent=4, ensure_ascii=False)
        f.write("\n")
    print(f"  wrote {os.path.relpath(path, ROOT)}")


def main():
    os.makedirs(VSCODE, exist_ok=True)
    print(f"[gen_vscode] detected: sdk={SDK} toolchain={TOOLCHAIN} openocd={OPENOCD} "
          f"picotool={PICOTOOL} cmake={CMAKE} ninja={NINJA}")
    print(f"[gen_vscode] target: board={BOARD} chip={CHIP}")
    write("settings.json", settings())
    write("tasks.json", tasks())
    write("launch.json", launch())
    write("extensions.json", extensions())
    print("[gen_vscode] done.")


if __name__ == "__main__":
    main()
