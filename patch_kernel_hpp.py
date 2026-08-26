import re

path = '/Users/ishigakiyua/github/Shizuku/internal_headers/shizuku/templates/kernel.hpp'
with open(path, 'r') as f:
    content = f.read()

content = content.replace(
    '  typename THREAD::state_t thread_state(uint32_t thread) const {',
    '  void set_thread_object(uint32_t thread, uint32_t obj) { m_threads[thread].thread.current_object = obj; }\n  typename THREAD::state_t thread_state(uint32_t thread) const {'
)

with open(path, 'w') as f:
    f.write(content)
