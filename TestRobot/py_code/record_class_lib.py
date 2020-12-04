from collections import namedtuple
from mas_utils import findConflictDictInfo

class Record:
    def __init__(self, robots, imap_graph):
        # robots: a list of Robot
        # imap_graph: nx.Graph()
        self.global_runtime = 0
        self.number_of_robots = len(robots)
        self.map_info = imap_graph

        self.current_step_info = []
        self.next_step_info = []
        self.record_book = {}
        self.conflict_robots = {}

        self.planned_next_step_conflict_flag = False

    def takeCurrentStepRecord(self, robots):
        self.current_step_info = []
        rbt_log = namedtuple('rbt_log', ['idx', 'current_pos', 'next_pos', 'target_pos'])
        for robot in robots:
            rbt = rbt_log(robot.idx, robot.current_position, robot.next_position, robot.target_position)
            self.current_step_info.append(rbt)
        self.record_book.update({self.global_runtime: self.current_step_info})
        # self.global_runtime += 1

    def conflictCheck(self):
        #rbt_log_list = list(self.record_book.items())[0][1] # a list of rbt_log data
        rbt_log_list = list(self.current_step_info)
        next_conflict_dict, current_position_dict = findConflictDictInfo(rbt_log_list)

        # check conflicts
        # # # # # # # 
        if (any((len(item[1]) > 1) for item in next_conflict_dict.items())): # two or more robots will take the same position in next step
            self.planned_next_step_conflict_flag = True
            # print("Conflict happens... Actions required")
            self.conflict_robots = [next_conflict_dict, current_position_dict]
            # return print(conflict_dict)
        else: # no next step conflicts
            # next takes non-self current check
            for item in next_conflict_dict.items():
                next_pos = item[0]
                idx = item[1][0]
                try:
                    if (current_position_dict[next_pos] != idx): # no current takes this next pos
                        # print("Planned pos takes other robot's current pos")
                        # print("robot " + str(idx) + " next_pos" + str(next_pos) + " takes" + " robot " + str(current_position_dict[next_pos]) + " current pos")
                        self.planned_next_step_conflict_flag = True
                        # print("Conflict happens... Actions required")
                        self.conflict_robots = [next_conflict_dict, current_position_dict]
                        break
                except KeyError:
                    pass
            else:
                print("Conflict-free paths found")
                self.planned_next_step_conflict_flag = False
                self.conflict_robots = [[], []]

    def push2Display(self):
        pos_text = str(self.number_of_robots) + "\n"

        for Rbt_log, i in zip(self.current_step_info, range(self.number_of_robots)):
            pos_text = pos_text + str(Rbt_log.idx + 1) + "," + str(Rbt_log.current_pos[0] + 1) + "," + str(Rbt_log.current_pos[1]+ 1) + "," + str(Rbt_log.target_pos[0]+ 1) + "," + str(Rbt_log.target_pos[0]+ 1) + "\n"

        with open("./../Robot_Current_Position.txt", "w") as f:
            f.write(pos_text)
        f.close()

    def takeValidRecord(self):
        self.record_book[self.global_runtime] = self.current_step_info
        self.global_runtime += 1

    def writeRecordBook2File(self):
        pass