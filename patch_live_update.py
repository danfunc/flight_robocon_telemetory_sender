#!/usr/bin/env python3
"""Apply patches to Shizuku kernel for Live Update / Dynamic Module execution."""

import re
import sys

# 1. Patch board.cpp: MPU Region 1 execute_never = false
board_path = "/Users/ishigakiyua/github/Shizuku/modules/pico_sdk_support/board.cpp"
with open(board_path, "r", encoding="utf-8") as f:
    board_content = f.read()

target_board = "ARCH_TYPE::region_set(1, heap, SRAM_END - 32u, ARCH_TYPE::ACCESS_RW_ALL, true,\n                        1);"
replacement_board = "ARCH_TYPE::region_set(1, heap, SRAM_END - 32u, ARCH_TYPE::ACCESS_RW_ALL, false,\n                        1); // ★動的モジュール実行のため XN=false"

if target_board in board_content:
    board_content = board_content.replace(target_board, replacement_board)
    with open(board_path, "w", encoding="utf-8") as f:
        f.write(board_content)
    print("[1/3] board.cpp MPU XN patched successfully.")
elif replacement_board in board_content:
    print("[1/3] board.cpp already patched.")
else:
    print("[1/3] WARNING: Could not find target in board.cpp")

# 2. Patch handler.cpp: create_object allows recreation / method update if flags & 0x80000000
handler_path = "/Users/ishigakiyua/github/Shizuku/source/kernel_object/handler.cpp"
with open(handler_path, "r", encoding="utf-8") as f:
    handler_content = f.read()

target_handler = "if (!m_objects[id].created) {\n      m_objects[id].created = true;\n      m_objects[id].flags = (uint32_t)flags;\n      // 最初のメソッドは生成側が与える (オブジェクト自身はまだ走っていないので\n      // 自分では登録できない)。以後は EXPORT_METHOD で自分が増やす。\n      m_objects[id].methods[0] = (method_t)entry;\n      return id;\n    }"
replacement_handler = "if (!m_objects[id].created || (flags & 0x80000000u) != 0) {\n      m_objects[id].created = true;\n      m_objects[id].flags = (uint32_t)(flags & ~0x80000000u);\n      // 最初のメソッドは生成側が与える (オブジェクト自身はまだ走っていないので\n      // 自分では登録できない)。以後は EXPORT_METHOD で自分が増やす。\n      m_objects[id].methods[0] = (method_t)entry;\n      return id;\n    }"

if target_handler in handler_content:
    handler_content = handler_content.replace(target_handler, replacement_handler)
    with open(handler_path, "w", encoding="utf-8") as f:
        f.write(handler_content)
    print("[2/3] handler.cpp create_object patched successfully.")
elif replacement_handler in handler_content:
    print("[2/3] handler.cpp already patched.")
else:
    print("[2/3] WARNING: Could not find target in handler.cpp")

# 3. Patch gdb_stub.cpp: pass 0x80000000 to create_object and add ISB / DMB barrier
gdb_stub_path = "/Users/ishigakiyua/github/Shizuku/modules/pico_sdk_support/objects/gdb_stub.cpp"
with open(gdb_stub_path, "r", encoding="utf-8") as f:
    gdb_content = f.read()

target_gdb = """      const auto c_res = api(object_api::CREATE_OBJECT, obj_id, entry_or_id, 0);
      if (c_res.error != 0 && c_res.error != (uintptr_t)object_error::ALREADY_EXISTS) {
        snprintf(line, sizeof(line), "create_object failed (%lu)\\n", (unsigned long)c_res.error);
        reply_hex_text(line);
        return;
      }"""

replacement_gdb = """      // 0x80000000 フラグで再利用時も methods[0] を新 entry_or_id に更新
      const auto c_res = api(object_api::CREATE_OBJECT, obj_id, entry_or_id, 0x80000000u);
      if (c_res.error != 0) {
        snprintf(line, sizeof(line), "create_object failed (%lu)\\n", (unsigned long)c_res.error);
        reply_hex_text(line);
        return;
      }
      asm volatile("dsb\\nisb" ::: "memory");"""

if target_gdb in gdb_content:
    gdb_content = gdb_content.replace(target_gdb, replacement_gdb)
    with open(gdb_stub_path, "w", encoding="utf-8") as f:
        f.write(gdb_content)
    print("[3/3] gdb_stub.cpp spawn recreation & ISB barrier patched successfully.")
elif replacement_gdb in gdb_content:
    print("[3/3] gdb_stub.cpp already patched.")
else:
    print("[3/3] WARNING: Could not find target in gdb_stub.cpp")
