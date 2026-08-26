import re

path = '/Users/ishigakiyua/github/Shizuku/source/kernel_object/handler.cpp'
with open(path, 'r') as f:
    content = f.read()

# Replace assignments to m_shadow[...].depth = 0
content = re.sub(r'(m_shadow\[(.*?)\]\.depth\s*=\s*0;)', r'\1 sync_thread_object(\2);', content)

# Replace shadow.depth++
content = re.sub(r'(shadow\.depth\+\+;)', r'\1 sync_thread_object(thread);', content)

# Replace shadow.depth--
content = re.sub(r'(shadow\.depth--;)', r'\1 sync_thread_object(thread);', content)

# Replace shadow.depth = ...
content = re.sub(r'(shadow\.depth\s*=\s*shadow\.depth\s*>=\s*pops\s*\?\s*shadow\.depth\s*-\s*\(uint32_t\)pops\s*:\s*0;)', r'\1 sync_thread_object(thread);', content)

# Replace shadow.depth += pops
content = re.sub(r'(shadow\.depth\s*\+=\s*\(uint32_t\)pops;)', r'\1 sync_thread_object(thread);', content)

# Also m_thread_object
content = re.sub(r'(m_thread_object\[(.*?)\]\s*=\s*(.*?);)', r'\1 sync_thread_object(\2);', content)

with open(path, 'w') as f:
    f.write(content)
