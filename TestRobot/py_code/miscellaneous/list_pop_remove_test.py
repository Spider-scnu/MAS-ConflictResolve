from mas_utils import findMaxDepth


def suboptSelect(arg):
    # generateSubopt()
    # find a non-conflict one

    pass


def addPos(arg):
    print(arg)
    pass



test_list = [
    [1],
    [2, 3],
    [4, 5, 6]
]

max_depth = findMaxDepth(test_list)
print("max depth is", max_depth)

for elimination_iter in range(0, max_depth):
    for inner_list in test_list:
        print(inner_list[0])
        inner_list.pop(0)

    while([] in test_list):
        test_list.remove([])
