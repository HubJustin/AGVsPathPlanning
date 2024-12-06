import copy
import random
import time
import math
import gym
from gym.envs.classic_control import rendering


class Goods:
    def __init__(self, x_point, y_point, x_dest, y_dest):
        self.x_point = x_point
        self.y_point = y_point
        self.x_dest = x_dest
        self.y_dest = y_dest
        self.active_status = False


class AGVs:
    def __init__(self, x_point, y_point):
        self.x_init_point = x_point
        self.y_init_point = y_point
        self.x_point = x_point
        self.y_point = y_point
        self.dest_pos = None
        self.move_operation = None
        self.collision = False
        self.status = None
        self.tasklist = []
        self.find_path_status = True

    def point_by_move(self, m_direction: tuple):
        if self.collision is False:
            self.x_point += m_direction[0]
            self.y_point += m_direction[1]


class Graph:
    def __init__(self, x_size, y_size, plaid_size=20, graph_bound=2):  # x_size=30, y_size=30
        self.x_size = x_size
        self.y_size = y_size
        self.plaid_size = plaid_size
        self.plaid_status = [['*' for _ in range(y_size)] for _ in range(x_size)]  # '*' or '#'
        self.dirs = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        self.graph_bound = graph_bound
        self.viewer = rendering.Viewer(x_size * plaid_size + 2 * graph_bound, y_size * plaid_size + 2 * graph_bound)

    def draw_grid(self):
        for x_index in range(self.x_size + 1):
            start_point = (x_index * self.plaid_size + self.graph_bound, self.graph_bound)
            end_point = (x_index * self.plaid_size + self.graph_bound, self.y_size * self.plaid_size + self.graph_bound)
            self.viewer.draw_line(start_point, end_point)
        for y_index in range(self.y_size + 1):
            start_point = (self.graph_bound, y_index * self.plaid_size + self.graph_bound)
            end_point = (self.x_size * self.plaid_size + self.graph_bound, y_index * self.plaid_size + self.graph_bound)
            self.viewer.draw_line(start_point, end_point)

    def draw_circle(self, x_point, y_point, color):
        x_circle = x_point * self.plaid_size - 0.5 * self.plaid_size + self.graph_bound
        y_circle = y_point * self.plaid_size - 0.5 * self.plaid_size + self.graph_bound
        self.viewer.draw_circle(8, 30, True, color=color).add_attr(
            rendering.Transform(translation=(x_circle, y_circle)))

    def draw_square(self, x_point, y_point, color=(0, 0, 0)):
        x_square = (x_point - 1) * self.plaid_size + self.graph_bound + 1
        y_square = (y_point - 1) * self.plaid_size + self.graph_bound + 1
        self.viewer.draw_polygon([(0, 0), (0, self.plaid_size - 3),
                                  (self.plaid_size - 3, self.plaid_size - 3),
                                  (self.plaid_size - 3, 0)], True, color=color).add_attr(
            rendering.Transform(translation=(x_square, y_square)))

    def draw_dotted_line(self, x_point, y_point, move_dirs: list):
        x_line = x_point * self.plaid_size - 0.5 * self.plaid_size + self.graph_bound
        y_line = y_point * self.plaid_size - 0.5 * self.plaid_size + self.graph_bound
        for i in range(len(move_dirs)):
            for width in range(self.plaid_size):
                x_line_start = x_line + width * move_dirs[i][0]
                y_line_start = y_line + width * move_dirs[i][1]
                x_line_end = x_line + (width + 1) * move_dirs[i][0]
                y_line_end = y_line + (width + 1) * move_dirs[i][1]
                if width % 4 == 0:
                    self.viewer.draw_line((x_line_start, y_line_start), (x_line_end, y_line_end))
            x_line = x_line + self.plaid_size * move_dirs[i][0]
            y_line = y_line + self.plaid_size * move_dirs[i][1]

    def generate_obstacles(self, x_aisle, y_aisle):
        self.plaid_status = [['#' for _ in range(self.y_size)] for _ in range(self.x_size)]
        for x_pos in range(self.x_size):
            for y_pos in range(self.y_size):
                if x_pos in x_aisle or y_pos in y_aisle:
                    self.plaid_status[x_pos][y_pos] = '*'

    def draw_obstacles(self):
        for x_pos in range(self.x_size):
            for y_pos in range(self.y_size):
                if self.plaid_status[x_pos][y_pos] == '#':
                    self.draw_square(x_pos + 1, y_pos + 1, color=(0.6, 0.6, 0.6))

    def modify_goods_pos(self, x_point, y_point):
        if self.plaid_status[x_point][y_point - 1] == '*':
            return x_point, y_point - 1
        else:
            return x_point, y_point + 1


class PathPlanning:
    def __init__(self, path_graph, departure, destination):
        self.path_graph = path_graph
        self.departure = departure
        self.destination = destination
        self.cur_pos = departure
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 当前位置四个方向的偏移量,即可以转动的四个方向
        self.path = []  # 存找到的路径
        self.move_ope = []  # 存每步的移动方向
        self.g_dict_layouts = {}
        self.g_dict_layouts_deep = {}  # 深度
        self.g_dict_layouts_fn = {}  # 启发函数值
        self.stack_layouts = []
        # if path_graph[self.departure[0]][self.departure[1]] == '#':
        #     print(f'Departure is on #')
        # if path_graph[self.destination[0]][self.destination[1]] == '#':
        #     print(f'Destination is on #')

    # def mark(self, pos):  # 给迷宫maze的位置pos标"."表示“倒过了”
    #     self.path_graph[pos[0]][pos[1]] = '.'

    def passable(self, pos):  # 检查迷宫maze的位置pos是否可通行
        return self.path_graph[pos[0]][pos[1]] == "*"

    # 启发函数
    def get_distance(self, pos):
        return math.sqrt((self.destination[0] - pos[0]) * (self.destination[0] - pos[0])) + math.sqrt(
            (self.destination[1] - pos[1]) * (self.destination[1] - pos[1]))
        # 曼哈顿距离

    def turn_best(self, pos, m_direction, next_pos_deep):
        next_pos = pos[0] + m_direction[0], pos[1] + m_direction[1]  # 注意修改后的维度
        # 存储fn启发函数值,A*算法
        fn = self.get_distance(next_pos) + next_pos_deep
        return next_pos, fn

    # 修改后的A*解决法
    def find_path_A(self):
        # 构建开始节点和表格
        find_solution_path = False
        self.g_dict_layouts[self.cur_pos] = -1
        self.g_dict_layouts_deep[self.cur_pos] = 1
        self.g_dict_layouts_fn[self.cur_pos] = 1 + self.get_distance(self.cur_pos)

        self.stack_layouts.append(self.cur_pos)  # 当前状态存入列表

        while len(self.stack_layouts) > 0:
            # self.mark(self.cur_pos)  # 标记走到的位置
            self.cur_pos = min(self.g_dict_layouts_fn, key=self.g_dict_layouts_fn.get)
            del self.g_dict_layouts_fn[self.cur_pos]
            self.stack_layouts.remove(self.cur_pos)  # 找到最小fn，并移除,扩展节点向下搜寻

            if self.cur_pos == self.destination:  # 判断当前状态是否为目标状态
                break
            for direction in self.directions:  # 当前可进行移动的四个方向，接下来对每个方向进行可行性判断
                new_pos, fn = self.turn_best(self.cur_pos, direction, self.g_dict_layouts_deep[self.cur_pos] + 1)
                if new_pos[0] < 0 or new_pos[1] < 0 or new_pos[0] >= len(self.path_graph) or new_pos[1] >= len(
                        self.path_graph[0]):
                    continue
                if self.g_dict_layouts.get(new_pos) is None:  # 判断交换后的状态是否已经查询过
                    if self.passable(new_pos):  # 不可行的相邻位置不管
                        self.g_dict_layouts_deep[new_pos] = self.g_dict_layouts_deep[self.cur_pos] + 1  # 存入深度
                        self.g_dict_layouts_fn[new_pos] = fn  # 存入fn
                        self.g_dict_layouts[new_pos] = self.cur_pos  # 定义前驱结点
                        self.stack_layouts.append(new_pos)  # 存入集合

        if self.cur_pos == self.destination:
            find_solution_path = True
        self.path.append(self.cur_pos)  # 存入每次找到的最优解决，之后可以不用g_dict_layouts表直接调用path
        while self.g_dict_layouts[self.cur_pos] != -1:  # 存入路径
            self.cur_pos = self.g_dict_layouts[self.cur_pos]
            self.path.append(self.cur_pos)
        self.path.reverse()
        return find_solution_path

    def get_move_operation(self):
        for index in range(len(self.path) - 1):
            self.move_ope.append(
                (self.path[index + 1][0] - self.path[index][0], self.path[index + 1][1] - self.path[index][1]))


class Env(gym.Env):
    # metadata = {
    #     'render.modes': ['human', 'rgb_array'],
    #     'video.frames_per_second': 2
    # }

    def __init__(self, goods_attr: dict, agv_attr: list, shelves_attr):
        self.graph = Graph(60, 40)
        self.graph.generate_obstacles(shelves_attr[0], shelves_attr[1])
        self.agv_color = [(1, 0.39, 0.28), (1, 0.55, 0), (0.93, 0.91, 0.67), (0.5, 0.5, 0), (1, 1, 0), (0.49, 0.99, 0),
                          (0, 0.5, 0), (0, 1, 0.5), (0, 1, 1), (1, 0, 1)]
        self.goods_clusters = [Goods(pos[0], pos[1], dest[0], dest[1]) for pos, dest in goods_attr.items()]
        self.goods_point = [(self.graph.modify_goods_pos(goods.x_point - 1, goods.y_point - 1)) for goods in
                            self.goods_clusters]
        self.goods_dest = [(self.graph.modify_goods_pos(goods.x_dest - 1, goods.y_dest - 1)) for goods in
                           self.goods_clusters]
        self.agv_clusters = [AGVs(agv[0], agv[1]) for agv in agv_attr]
        self.state = None
        self.agv_collision = [[] for _ in range(len(self.agv_clusters))]
        self.agv_priority = [[j for j in range(i + 1, len(self.agv_clusters))] for i in range(len(self.agv_clusters))]

    def detect_collision(self, reverse=False):
        # 碰撞类型: 对撞、交叉撞、追尾
        collision = False
        agv_num = len(self.agv_clusters)
        for i in range(agv_num - 1):
            agv_one = self.agv_clusters[i]
            if len(agv_one.move_operation) == 0 or agv_one.collision is True:
                agv_one_move_ope = (0, 0)
            else:
                agv_one_move_ope = agv_one.move_operation[0]
            for j in range(i + 1, agv_num):
                agv_two = self.agv_clusters[j]
                if len(agv_two.move_operation) == 0 or agv_two.collision is True:
                    agv_two_move_ope = (0, 0)
                else:
                    agv_two_move_ope = agv_two.move_operation[0]
                collision_type_1, collision_type_2, collision_type_3 = False, False, False
                position_type_1 = abs(agv_one.x_point - agv_two.x_point) == 1 and agv_one.y_point == agv_two.y_point
                position_type_2 = abs(agv_one.x_point - agv_two.x_point) == 2 and agv_one.y_point == agv_two.y_point
                position_type_3 = abs(agv_one.y_point - agv_two.y_point) == 1 and agv_one.x_point == agv_two.x_point
                position_type_4 = abs(agv_one.y_point - agv_two.y_point) == 2 and agv_one.x_point == agv_two.x_point
                position_type_5 = abs(agv_one.x_point - agv_two.x_point) == 1 and abs(
                    agv_one.y_point - agv_two.y_point) == 1
                if position_type_1:
                    collision_type_1 = (agv_one.x_point + agv_one_move_ope[
                        0] == agv_two.x_point or agv_one.x_point == agv_two.x_point + agv_two_move_ope[0])
                if position_type_3:
                    collision_type_2 = (agv_one.y_point + agv_one_move_ope[
                        1] == agv_two.y_point or agv_one.y_point == agv_two.y_point + agv_two_move_ope[1])
                if position_type_2 or position_type_4 or position_type_5:
                    collision_condition_1 = agv_one.x_point + agv_one_move_ope[0] == agv_two.x_point + agv_two_move_ope[
                        0]
                    collision_condition_2 = agv_one.y_point + agv_one_move_ope[1] == agv_two.y_point + agv_two_move_ope[
                        1]
                    collision_type_3 = collision_condition_1 and collision_condition_2
                if collision_type_1 or collision_type_2 or collision_type_3:
                    agv_one.collision = True
                    agv_two.collision = True
                    collision = True
                    if j in self.agv_priority[i]:
                        self.agv_collision[i].append(j)
                        self.agv_priority[i].remove(j)
                        self.agv_priority[j].append(i)
                    else:
                        self.agv_collision[j].append(i)
                        self.agv_priority[i].append(j)
                        self.agv_priority[j].remove(i)

        return collision

    def re_planning_path(self):
        if sum([len(collision) for collision in self.agv_collision]) != 0:
            for collision in self.agv_collision:
                if len(collision) == 0:
                    continue
                else:
                    collision_min_id = collision[0]
                    graph_plaid_status = copy.deepcopy(self.graph.plaid_status)
                    agv_collision_id = self.agv_collision.index(collision)
                    for agv_index in collision:
                        agv = self.agv_clusters[agv_index]
                        agv.collision = False
                        # if agv.move_operation[0] != (0, 0):
                        if len(self.agv_clusters[agv_collision_id].move_operation) > 0:
                            # if self.agv_clusters[agv_collision_id].dest_pos[0] == (agv.x_point - 1) and \
                            #         self.agv_clusters[agv_collision_id].dest_pos[1] == (agv.y_point - 1):
                            #     pass
                            # else:
                            agv.move_operation.insert(0, (0, 0))
                        # if agv.move_operation.count((0, 0)) > 1:
                        #     print(f'agv_id: {agv_index}, num: {agv.move_operation.count((0, 0))}')
                        graph_plaid_status[agv.x_point - 1][agv.y_point - 1] = '#'
                        graph_plaid_status[agv.x_point + agv.move_operation[0][0] - 1][
                            agv.y_point + agv.move_operation[0][1] - 1] = '#'
                    del self.agv_collision[agv_collision_id][:]
                    agv_collision = self.agv_clusters[agv_collision_id]
                    # move_ope_key = [move_ope for move_ope in agv_collision.move_operation if
                    #                 move_ope[0] == 0 and move_ope[1] == 0]
                    move_ope_key = []
                    if len(agv_collision.move_operation) > 0:
                        if agv_collision.move_operation[0][0] == 0 and agv_collision.move_operation[0][1] == 1:
                            move_ope_key.append((0, 0))
                    pathplanning = PathPlanning(graph_plaid_status,
                                                (agv_collision.x_point - 1, agv_collision.y_point - 1),
                                                agv_collision.dest_pos)
                    agv_collision.find_path_status = pathplanning.find_path_A()
                    pathplanning.get_move_operation()
                    if len(pathplanning.move_ope) > 0:
                        # if agv_collision_id > collision_min_id:
                        #     self.agv_clusters[agv_collision_id].move_operation = pathplanning.move_ope
                        # else:
                        self.agv_clusters[agv_collision_id].move_operation = move_ope_key + pathplanning.move_ope
                    agv_collision.collision = False

    def get_feature(self):
        pass

    def assign_tasks(self):
        agv_num = len(self.agv_clusters)
        for goods_id in range(len(self.goods_clusters)):
            assign_agv_id = goods_id % agv_num
            self.agv_clusters[assign_agv_id].tasklist.extend([self.goods_point[goods_id], self.goods_dest[goods_id]])
        for agv in self.agv_clusters:
            agv.tasklist.append((agv.x_init_point - 1, agv.y_init_point - 1))

    def step(self):
        # goods_point = self.goods_point
        # goods_dest = self.goods_dest
        self.assign_tasks()
        for agv_id in range(len(self.agv_clusters)):
            agv = self.agv_clusters[agv_id]
            # agv.tasklist = [goods_point[agv_id], goods_dest[agv_id]]
            agv.dest_pos = agv.tasklist[0]
            del agv.tasklist[0]
            pathplanning = PathPlanning(self.graph.plaid_status, (agv.x_point - 1, agv.y_point - 1), agv.dest_pos)
            agv.find_path_status = pathplanning.find_path_A()
            pathplanning.get_move_operation()
            self.agv_clusters[agv_id].move_operation = pathplanning.move_ope
        self.render()
        time.sleep(1)
        # index = 0
        agv_max_move_ope = max([len(agv.move_operation) for agv in self.agv_clusters])
        # while index < agv_max_move_ope:
        while True:
            while self.detect_collision():
                self.re_planning_path()
            # if self.detect_collision():
            #     self.re_planning_path()
            #     self.detect_collision()
            #     self.re_planning_path()
            #     self.detect_collision()
            #     self.re_planning_path()
                # self.detect_collision()

                # self.agv_clusters.reverse()
            # re = self.detect_collision()
            # self.re_planning_path()
            for agv in self.agv_clusters:
                if len(agv.move_operation) > 0:
                    if agv.collision:
                        print(f'AGV_{self.agv_clusters.index(agv)}')
                        continue
                    agv.point_by_move(agv.move_operation[0])
                    del agv.move_operation[0]
                    # if (agv.x_point - 1) == agv.dest_pos[0] and (agv.y_point - 1) == agv.dest_pos[1]:
                    #     del agv.tasklist[0]
                    # if (agv.x_point - 1, agv.y_point - 1) == agv.dest_pos:
                    #     del agv.tasklist[0]
                # elif agv.status is None:
                if agv.find_path_status is False:
                    pathplanning = PathPlanning(self.graph.plaid_status, (agv.x_point - 1, agv.y_point - 1),
                                                agv.dest_pos)
                    agv.find_path_status = pathplanning.find_path_A()
                    pathplanning.get_move_operation()
                    agv.move_operation = pathplanning.move_ope
                if len(agv.move_operation) == 0 and len(agv.tasklist) > 0:
                    # agv.status = 0
                    agv_id = self.agv_clusters.index(agv)
                    agv.dest_pos = agv.tasklist[0]
                    del agv.tasklist[0]
                    pathplanning = PathPlanning(self.graph.plaid_status, (agv.x_point - 1, agv.y_point - 1),
                                                agv.dest_pos)
                    agv.find_path_status = pathplanning.find_path_A()
                    pathplanning.get_move_operation()
                    agv.move_operation.extend(pathplanning.move_ope)
                if len(agv.move_operation) == 0 and len(agv.tasklist) == 0:
                    self.graph.plaid_status[agv.x_point - 1][agv.y_point - 1] = '#'
                    # if agv.collision:
                    #     continue
                    # agv.point_by_move(agv.move_operation[0])
                    # del agv.move_operation[0]
            # [agv.point_by_move(agv.move_operation[index]) for agv in self.agv_clusters if index < len(agv.move_operation)]
            # if self.graph.plaid_status[self.agv_clusters[0].x_point + m_direction[0] - 1][self.agv_clusters[0].y_point +
            #                                                                               m_direction[1] - 1] == '#':
            #     pathplanning = PathPlanning(self.graph.plaid_status,
            #                                 (self.agv_clusters[0].x_point - 1, self.agv_clusters[0].y_point - 1),
            #                                 (45, 32))
            #     pathplanning.find_path_A()
            #     pathplanning.get_move_operation()
            #     print(pathplanning.path)
            #     print(pathplanning.move_ope)
            #     self.agv_clusters[0].move_operation[index:] = pathplanning.move_ope
            #     continue
            # self.agv_clusters[0].point_by_move(m_direction)
            # time.sleep(0.1)
            self.render()
            agv_finish = True
            for agv in self.agv_clusters:
                if agv_finish is True:
                    agv_finish = (self.graph.plaid_status[agv.x_init_point - 1][agv.y_init_point - 1] == '#')
                else:
                    break
            if agv_finish is True:
                break

    def reset(self):
        pass

    def render(self, mode='human'):
        # 下面就可以定义你要绘画的元素了
        self.graph.draw_grid()
        self.graph.draw_obstacles()
        for agv in self.agv_clusters:
            agv_id = self.agv_clusters.index(agv)
            # if agv_id == 2 or agv_id == 5:
            # self.graph.draw_dotted_line(agv.x_point, agv.y_point, agv.move_operation)
            # self.graph.draw_circle(agv.x_point, agv.y_point, (1, 0, 0) if agv.collision else (0, 0, 1))
            self.graph.draw_circle(agv.x_point, agv.y_point, (1, 0, 0) if agv.collision else self.agv_color[agv_id])

            # self.graph.draw_circle(agv.x_point, agv.y_point, (0, 0.5, 0.5) if agv.status == 0 else (0, 0, 1))
        # for goods in self.goods_clusters:
        #     self.graph.draw_circle(goods.x_point, goods.y_point, (0.5, 0, 0))
        #     self.graph.draw_circle(goods.x_dest, goods.y_dest, (0, 1, 0))
        # self.graph.draw_circle(agv.dest_pos[0]+1, agv.dest_pos[1]+1, (0, 1, 0))
        return self.graph.viewer.render(return_rgb_array=mode == 'rgb_array')

    def close(self):
        if self.graph.viewer:
            self.graph.viewer.close()


if __name__ == '__main__':
    agv_info = [(17, 1), (20, 1), (23, 1), (26, 1), (29, 1), (32, 1), (35, 1), (38, 1), (41, 1), (44, 1)]
    x_aisle = [0, 7, 8, 15, 16, 17, 24, 25, 32, 33, 34, 41, 42, 49, 50, 51, 58, 59]
    y_aisle = [0, 1, 2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 39]
    # goods_info = {(46, 32): (4, 5), (46, 38): (7, 14), (10, 35): (54, 38), (7, 4): (40, 22), (13, 13): (28, 32),
    #               (28, 22): (4, 4), (40, 4): (10, 13), (47, 14): (13, 32), (54, 29): (28, 4), (28, 34): (47, 22),
    #               (27, 19): (58, 38)}
    x_shelves = [i + 1 for i in range(60) if i not in x_aisle]
    y_shelves = [i + 1 for i in range(40) if i not in y_aisle]
    # for i in range(195, 1000):
    #     time_start = time.time()
    #     time_now = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
    #     print(f'random_seed: {i}, the start time: {time_now},', end=' ')
    random.seed(100)  # seed = 148, 174, 190, 194
    x_random_id = [random.randint(0, len(x_shelves) - 1) for _ in range(4000)]
    y_random_id = [random.randint(0, len(y_shelves) - 1) for _ in range(4000)]
    pos_info = [(x_shelves[x_random_id[i]], y_shelves[y_random_id[i]]) for i in range(4000)]
    goods_info = {}
    for pos_id in range(int(len(pos_info) / 2)):
        pos_id_one = pos_id
        pos_id_two = pos_id + int(len(pos_info) / 2)
        if pos_info[pos_id_one][0] == pos_info[pos_id_two][0] and pos_info[pos_id_one][1] == pos_info[pos_id_two][1]:
            continue
        goods_info[pos_info[pos_id_one]] = pos_info[pos_id_two]
    # index = 0
    # for key, value in goods_info.items():
    #     index += 1
    #     print(f'{index}, AGV_{index % 10}, {key}: {value}')
    # print(len(goods_info))
    env = Env(goods_info, agv_info, (x_aisle, y_aisle))
    env.step()
    env.close()
        # print(f'the token time: {time.time()-time_start}s')
