a = [[0,1,2,3], [4,5],[6]]
b = [2*x for x in range(7)]
c = [b[x[0]:(x[-1]+1)] for x in a]
print(b)
print(a)
print(c)