b = ['node', 'node', 'node', 'node', 'leader']
word = ['leader']
c = [i for i, x in enumerate(b) if x in word]
print(c)