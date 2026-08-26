path = '/Users/ishigakiyua/github/Shizuku/internal_headers/shizuku/templates/kernel_object.hpp'
with open(path, 'r') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if 'shadow.depth' in line and ('++' in line or '--' in line or '+' in line or '-' in line or '=' in line):
        print(f"--- Line {i+1} ---")
        for j in range(max(0, i-5), min(len(lines), i+6)):
            print(f"{j+1}: {lines[j]}", end='')
