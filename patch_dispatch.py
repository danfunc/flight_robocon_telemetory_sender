import re

path = '/Users/ishigakiyua/github/Shizuku/source/kernel/dispatch.cpp'
with open(path, 'r') as f:
    content = f.read()

content = content.replace(
    'if ((thread.call_stack.depth & 1u) == 0) {',
    'if (thread.current_object != 0) { // 0 = ROOT_OBJECT (KERNEL_OBJECT)'
)

content = content.replace(
    'if ((current_depth() & 1u) != 0u) {',
    'if (current_thread().current_object == 0) { // 0 = ROOT_OBJECT'
)

with open(path, 'w') as f:
    f.write(content)
