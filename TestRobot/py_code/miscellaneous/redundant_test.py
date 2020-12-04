next_position_list = [(0, 1), (0, 1), (1, 3), (1, 3), (4, 4), (4, 4), (12, 3), (16, 7)]
npl_set = set(next_position_list)

conflict_dict = {}

for item in npl_set:
    indices = [i for i, pos in enumerate(next_position_list) if pos == item]
    conflict_dict.update({item: indices})

for item in conflict_dict.items():
    print(item)