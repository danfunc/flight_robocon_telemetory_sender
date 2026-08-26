import re

path = '/Users/ishigakiyua/github/Shizuku/internal_headers/shizuku/templates/kernel_object.hpp'
with open(path, 'r') as f:
    content = f.read()

content = content.replace(
    '  uintptr_t current_object(uint32_t thread) const {',
    '  void sync_thread_object(uint32_t thread) {\n    kernel_instance.set_thread_object(thread, current_object(thread));\n  }\n  uintptr_t current_object(uint32_t thread) const {'
)

with open(path, 'w') as f:
    f.write(content)
