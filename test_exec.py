import sys
import time
sys.path.append('tools')
import shizuku_hot_reload as shr

rsp = shr.SerialRspClient('/dev/cu.usbmodem1103')
alloc_resp = rsp.qrcmd("alloc 32")
ram_addr = int(alloc_resp.strip(), 16)
print(f"Allocated at {hex(ram_addr)}")

# Write a simple bkpt #0 (0xbe00)
bkpt_insn = b"\x00\xbe"
rsp.write_mem(ram_addr, bkpt_insn)

entry = ram_addr | 1
print(f"Spawning at {hex(entry)}")
print(rsp.qrcmd(f"spawn 0x{entry:08x}"))
