lst = [0, 1, 2, 3, 4, 5]

for i in range(10):
    try:
        next_ele = lst[1]
        lst.pop(0)
    except IndexError:
        next_ele = lst[0]
    print(next_ele)

