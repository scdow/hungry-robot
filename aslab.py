import numpy as np

aslist = [5,2,7,10]
print(aslist[0:2])

as_dot = np.diff(aslist)
print(len(as_dot), as_dot)

a, b =0, 0
a +=5
b -=2
print(a,b)

def test_main():
    print('main')

print(3/2.1)

print(np.mean(aslist))

bslist=[1,'a']
bslist.append(aslist)
print(bslist)

clist = [1,2,3]
clist = [10,11]
print(clist)

if __name__ == '__main__':
    test_main() # use this line to run evolution
    # rerun() # use this line to simulate a winner from a previous evolutionary run which has been saved to file
