path = '/Users/ishigakiyua/github/Shizuku/source/kernel/dispatch.cpp'
with open(path, 'r') as f:
    lines = f.readlines()
for i in range(110, 140):
    print(f"{i+1}: {lines[i]}", end='')
for i in range(235, 245):
    print(f"{i+1}: {lines[i]}", end='')
