import re

# Revert kernel_object.hpp
path1 = '/Users/ishigakiyua/github/Shizuku/internal_headers/shizuku/templates/kernel_object.hpp'
with open(path1, 'r') as f:
    content1 = f.read()

content1 = content1.replace(
    '  void sync_thread_object(uint32_t thread) {\n    kernel_instance.set_thread_object(thread, current_object(thread));\n  }\n  uintptr_t current_object(uint32_t thread) const {',
    '  uintptr_t current_object(uint32_t thread) const {'
)

with open(path1, 'w') as f:
    f.write(content1)

# Patch handler.cpp to use kernel_instance.set_thread_object
path2 = '/Users/ishigakiyua/github/Shizuku/source/kernel_object/handler.cpp'
with open(path2, 'r') as f:
    content2 = f.read()

content2 = content2.replace('sync_thread_object(', 'kernel_instance.set_thread_object(')
# Wait! set_thread_object expects 2 args! (thread, object_id)
# My previous sync_thread_object took 1 arg (thread).
# So I need to replace 'kernel_instance.set_thread_object(thread);' 
# with 'kernel_instance.set_thread_object(thread, current_object(thread));'

content2 = re.sub(
    r'kernel_instance\.set_thread_object\((.*?)\);',
    r'kernel_instance.set_thread_object(\1, current_object(\1));',
    content2
)

with open(path2, 'w') as f:
    f.write(content2)
