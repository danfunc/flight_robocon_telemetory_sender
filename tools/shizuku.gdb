# Shizuku GDB Helper Macros (Native GDB script)
# どの GDB (Python 有無問わず) でも動作するマクロ定義

define shizuku-help
  printf "=== Shizuku GDB Commands ===\n"
  printf "  monitor list                  - 動作中スレッド一覧表示\n"
  printf "  monitor target <id|name>      - デバッグ対象スレッドの選択\n"
  printf "  monitor alloc <bytes>         - 実機RAMに動的メモリ確保 (例: monitor alloc 64)\n"
  printf "  monitor free <addr>           - 実機動的メモリ解放 (例: monitor free 0x20015000)\n"
  printf "  monitor spawn <addr|id> [arg] - 動的スレッド起動 (例: monitor spawn blink_main)\n"
  printf "  monitor call <id> <m> [arg]   - オブジェクトメソッド同期実行 (例: monitor call 2 0 1)\n"
  printf "============================\n"
end

define shizuku-alloc
  if $argc == 1
    monitor alloc $arg0
  else
    printf "使用法: shizuku-alloc <bytes>\n"
  end
end

define shizuku-free
  if $argc == 1
    monitor free $arg0
  else
    printf "使用法: shizuku-free <addr>\n"
  end
end

define shizuku-spawn
  if $argc == 1
    monitor spawn $arg0
  else
    if $argc == 2
      monitor spawn $arg0 $arg1
    else
      printf "使用法: shizuku-spawn <entry_addr_or_id> [arg]\n"
    end
  end
end

define shizuku-call
  if $argc == 2
    monitor call $arg0 $arg1
  else
    if $argc == 3
      monitor call $arg0 $arg1 $arg2
    else
      printf "使用法: shizuku-call <obj_id> <method_id> [arg]\n"
    end
  end
end
