import re
import sys

filepath = '/Users/ishigakiyua/github/Shizuku/modules/pico_sdk_support/objects/gdb_stub.cpp'

with open(filepath, 'r') as f:
    content = f.read()

# Replace 100000 with 2000000 in wait_for_agents
new_content = re.sub(r'for \(uint32_t guard = 0; guard < 100000; \+\+guard\) \{', r'for (uint32_t guard = 0; guard < 2000000; ++guard) {', content)

if new_content == content:
    print("No changes made. Patch failed.")
    sys.exit(1)

with open(filepath, 'w') as f:
    f.write(new_content)

print("Patch applied successfully.")
