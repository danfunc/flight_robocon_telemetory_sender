path = '/Users/ishigakiyua/github/Shizuku/source/kernel_object/handler.cpp'
with open(path, 'r') as f:
    lines = f.readlines()
for i in range(380, 410):
    print(f"{i+1}: {lines[i]}", end='')
