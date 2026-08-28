#!/usr/bin/env python3
"""Shizuku GDB Python Extension

GDB 内で実機（RP2350）上の動的メモリ確保・データ注入・動的スレッド生成を行うコマンド群。

使い方:
  (gdb) source tools/shizuku_gdb.py
  (gdb) shizuku-alloc my_buf 64
  (gdb) shizuku-alloc gain float
  (gdb) set $gain = 2.5
  (gdb) shizuku-spawn blink_main
  (gdb) shizuku-free $my_buf
"""

try:
    import gdb
except ImportError:
    pass


class ShizukuAlloc(gdb.Command):
    """実機RAM上に動的にメモリを確保し、GDB変数 ($<name>) に束縛します。
    使用法: shizuku-alloc <変数名> <バイト数または型名>
    例:
      shizuku-alloc my_buf 128
      shizuku-alloc gain float
      shizuku-alloc config uint32_t[4]
    """

    def __init__(self):
        super().__init__("shizuku-alloc", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        parts = arg.strip().split(None, 1)
        if not parts:
            print("使用法: shizuku-alloc <変数名> <バイト数または型名>")
            return
        name = parts[0].lstrip("$")
        type_or_size = parts[1] if len(parts) > 1 else "4"

        # バイト数の計算 (数値ならそのまま、型名なら sizeof(型))
        size_bytes = 4
        cast_type = "char*"
        try:
            size_bytes = int(type_or_size, 0)
            cast_type = "char*"
        except ValueError:
            try:
                gdb_type = gdb.lookup_type(type_or_size)
                size_bytes = gdb_type.sizeof
                cast_type = f"{type_or_size}*"
            except Exception:
                cast_type = f"{type_or_size}*"
                size_bytes = 4

        res = gdb.execute(f"monitor alloc {size_bytes}", to_string=True)
        lines = [line.strip() for line in res.splitlines() if line.strip().startswith("0x")]
        if not lines:
            print(f"[Shizuku GDB] メモリ確保に失敗しました: {res.strip()}")
            return
        addr_str = lines[0]
        gdb.execute(f"set ${name} = ({cast_type}){addr_str}")
        print(f"[Shizuku GDB] 実機メモリ確保成功: ${name} = ({cast_type}){addr_str} ({size_bytes} bytes)")


class ShizukuFree(gdb.Command):
    """実機RAM上に確保した動的メモリを解放します。
    使用法: shizuku-free <アドレスまたはGDB変数>
    例:
      shizuku-free $my_buf
      shizuku-free 0x20015000
    """

    def __init__(self):
        super().__init__("shizuku-free", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        arg = arg.strip()
        if not arg:
            print("使用法: shizuku-free <アドレスまたはGDB変数>")
            return
        try:
            val = gdb.parse_and_eval(arg)
            addr = int(val)
        except Exception:
            try:
                addr = int(arg, 0)
            except Exception:
                print(f"[Shizuku GDB] アドレスを解釈できません: {arg}")
                return

        res = gdb.execute(f"monitor free {hex(addr)}", to_string=True)
        print(f"[Shizuku GDB] {res.strip()}")


class ShizukuSpawn(gdb.Command):
    """指定した関数またはオブジェクトIDから動的に新しいスレッドを生成・起動します。
    使用法: shizuku-spawn <関数名またはアドレス> [引数]
    例:
      shizuku-spawn blink_main
      shizuku-spawn 0x10005480 1234
    """

    def __init__(self):
        super().__init__("shizuku-spawn", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        parts = arg.strip().split()
        if not parts:
            print("使用法: shizuku-spawn <関数名またはアドレス> [引数]")
            return
        target = parts[0]
        arg1 = parts[1] if len(parts) > 1 else "0"

        addr = 0
        try:
            val = gdb.parse_and_eval(target)
            addr = int(val)
        except Exception:
            try:
                addr = int(target, 0)
            except Exception:
                print(f"[Shizuku GDB] シンボルまたはアドレスが見つかりません: {target}")
                return

        res = gdb.execute(f"monitor spawn {hex(addr)} {arg1}", to_string=True)
        print(f"[Shizuku GDB] {res.strip()}")


class ShizukuCall(gdb.Command):
    """オブジェクトのメソッドを同期実行して戻り値を取得します。
    使用法: shizuku-call <オブジェクトID> <メソッドID> [引数]
    例:
      shizuku-call 2 0 1
    """

    def __init__(self):
        super().__init__("shizuku-call", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        parts = arg.strip().split()
        if len(parts) < 2:
            print("使用法: shizuku-call <オブジェクトID> <メソッドID> [引数]")
            return
        obj_id = parts[0]
        method_id = parts[1]
        arg_val = parts[2] if len(parts) > 2 else "0"
        res = gdb.execute(f"monitor call {obj_id} {method_id} {arg_val}", to_string=True)
        print(f"[Shizuku GDB] {res.strip()}")


# 拡張コマンド登録
if "gdb" in globals():
    ShizukuAlloc()
    ShizukuFree()
    ShizukuSpawn()
    ShizukuCall()
    print("[Shizuku GDB] Shizuku dynamic extension loaded: shizuku-alloc, shizuku-free, shizuku-spawn, shizuku-call")
