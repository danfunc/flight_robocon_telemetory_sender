import re

path = '/Users/ishigakiyua/github/Shizuku/internal_headers/shizuku/templates/thread.hpp'
with open(path, 'r') as f:
    content = f.read()

content = content.replace(
    'uint32_t affinity = 0b1; // bit0 = core0 (どのコアで走ってよいか)',
    'uint32_t affinity = 0b1; // bit0 = core0 (どのコアで走ってよいか)\n  uint32_t current_object = 0;'
)

with open(path, 'w') as f:
    f.write(content)
