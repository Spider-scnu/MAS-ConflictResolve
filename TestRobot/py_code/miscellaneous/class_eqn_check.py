class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def tuple2Point(tp):
    return Point(tp[0], tp[1])


tuple_list = [(1, 3), (2, 4), (2, 4), (4, 6)]
point_list = []

for tl in tuple_list:
    point_list.append(tuple2Point(tl))

for tl in tuple_list:
    num_count = tuple_list.count(tl)
    if num_count >= 2:
        print("Duplication found:", tl)
        break


contain_duplicates = any(tuple_list.count(element) for element in tuple_list)
print(contain_duplicates)
