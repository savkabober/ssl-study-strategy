"""
Модуль вспомогательной математики и утилит
"""

import math
import typing

import bridge.processors.const as const
import bridge.processors.robot as robot


class Graph:
    """
    Класс для работы с графами
    """

    def __init__(self, num_vertices: int) -> None:
        """
        Конструктор

        Аллоцирует память под граф с num_vertices вершинами
        """
        self.num_vertices = num_vertices
        self.graph = [[0] * num_vertices for _ in range(num_vertices)]

    def add_edge(self, from_vertex: int, to_vertex: int, weight: int) -> None:
        """
        Добавить ребро графу
        """
        self.graph[from_vertex][to_vertex] = weight
        self.graph[to_vertex][from_vertex] = weight

    def dijkstra(self, start_vertex: int) -> list[float]:
        """
        Найти кратчайший путь в графе используя алгоритм Дейкстры
        """
        distances = [float("inf")] * self.num_vertices
        distances[start_vertex] = 0
        visited = [False] * self.num_vertices

        for _ in range(self.num_vertices):
            min_distance = float("inf")
            min_vertex = -1

            for v in range(self.num_vertices):
                if not visited[v] and distances[v] < min_distance:
                    min_distance = distances[v]
                    min_vertex = v

            visited[min_vertex] = True

            for v in range(self.num_vertices):
                if (
                    not visited[v]
                    and self.graph[min_vertex][v]
                    and distances[min_vertex] != float("inf")
                    and distances[min_vertex] + self.graph[min_vertex][v] < distances[v]
                ):
                    distances[v] = distances[min_vertex] + self.graph[min_vertex][v]

        return distances


class Point:
    """
    Класс, описывающий точку (вектор)
    """

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, p):
        return Point(self.x + p.x, self.y + p.y)

    def __neg__(self):
        return Point(-self.x, -self.y)

    def __sub__(self, p):
        return self + -p

    def __mul__(self, a: float):
        return Point(self.x * a, self.y * a)

    def __truediv__(self, a: float):
        return self * (1 / a)

    def __pow__(self, a: float):
        return Point(self.x**a, self.y**a)

    def __eq__(self, p):
        return self.x == p.x and self.y == p.y

    def __str__(self):
        return f"x = {self.x:.2f}, y = {self.y:.2f}"

    def mag(self):
        """
        Получить модуль вектора
        """
        return math.hypot(self.x, self.y)

    def arg(self):
        """
        Получить аргумент вектора (угол относительно оси OX)
        """
        return math.atan2(self.y, self.x)

    def unity(self):
        """
        Получить единичный вектор, коллинеарный данному
        """
        if self.mag() == 0:
            # raise ValueError("БАГА, .unity от нулевого вектора")
            return self
        return self / self.mag()


RIGHT = Point(1, 0)
UP = Point(0, 1)
GRAVEYARD_POS = Point(const.GRAVEYARD_POS_X, 0)


class BobLine:
    """
    Борина линия
    Надо ли?
    """

    def __init__(self, A: float, B: float, C: float):
        self.A = A
        self.B = B
        self.C = C


def dist2line(p1: Point, p2: Point, p: Point) -> float:
    """
    Рассчитать расстояние от точки p до прямой, образованной точками p1 и p2
    """
    return abs(vect_mult((p2 - p1).unity(), p - p1))


def line_poly_intersect(p1: Point, p2: Point, points: list[Point]) -> bool:
    """
    Определить, пересекает ли линия p1-p2 полигон points
    """
    vec = p2 - p1
    old_sign = sign(vect_mult(vec, points[0] - p1))
    for p in points:
        if old_sign != sign(vect_mult(vec, p - p1)):
            return True
    return False


def segment_poly_intersect(p1: Point, p2: Point, points: list[Point]) -> typing.Optional[Point]:
    """
    Определить, пересекает ли отрезок p1-p2 полигон points
    Если да - возвращает одну из двух точек пересечения
    Если нет - возвращает None
    """
    for i in range(-1, len(points) - 1):
        p = get_line_intersection(p1, p2, points[i], points[i + 1], "SS")
        if p is not None:
            return p
    return None


def is_point_inside_poly(p: Point, points: list[Point]) -> bool:
    """
    Определить, лежит ли точка внутри полигона
    """
    old_sign = sign(vect_mult(p - points[-1], points[0] - points[-1]))
    for i in range(len(points) - 1):
        if old_sign != sign(vect_mult(p - points[i], points[i + 1] - points[i])):
            return False
    return True


def dist(a: Point, b: Point) -> float:
    """
    Определить расстояние между двумя точками
    """
    return math.hypot(a.x - b.x, a.y - b.y)


def get_line_intersection(
    line1_start: Point, line1_end: Point, line2_start: Point, line2_end: Point, is_inf: str = "SS"
) -> typing.Optional[Point]:
    """
    Получить точку пересечения отрезков или прямых

    is_inf задает ограничения на точку пересечения. Имеет вид 'AB', параметр A
    задает параметры первой прямой, B - второй.

    S(egment) - задан отрезок
    R(ay) - задан луч (начало - _start, направление - _end), точка пересечения валидна только
    если находится на луче _start-_end
    L(ine) - задана прямая
    """
    # Calculate the differences
    delta_x1 = line1_end.x - line1_start.x
    delta_y1 = line1_end.y - line1_start.y
    delta_x2 = line2_end.x - line2_start.x
    delta_y2 = line2_end.y - line2_start.y

    # Calculate the determinants
    determinant = delta_y1 * delta_x2 - delta_y2 * delta_x1

    if determinant == 0:
        # The lines are parallel or coincident
        return None

    # Calculate the differences between the start points
    delta_x_start = line1_start.x - line2_start.x
    delta_y_start = line1_start.y - line2_start.y

    # Calculate the t parameters
    t1 = (delta_x_start * delta_y2 - delta_x2 * delta_y_start) / determinant
    t2 = (delta_x_start * delta_y1 - delta_x1 * delta_y_start) / determinant

    intersection_x = line1_start.x + t1 * delta_x1
    intersection_y = line1_start.y + t1 * delta_y1
    p = Point(intersection_x, intersection_y)

    first_valid = False
    second_valid = False
    if is_inf[0] == "S" and 0 <= t1 <= 1 or is_inf[0] == "R" and t1 >= 0 or is_inf[0] == "L":
        first_valid = True
    if is_inf[1] == "S" and 0 <= t2 <= 1 or is_inf[1] == "R" and t2 >= 0 or is_inf[1] == "L":
        second_valid = True

    if first_valid and second_valid:
        return p

    return None


def vect_mult(v: Point, u: Point) -> float:
    """
    Посчитать модуль векторного произведения векторов v и u
    """
    return v.x * u.y - v.y * u.x


def scal_mult(v: Point, u: Point) -> float:
    """
    Посчитать скалярное произведение векторов v и u
    """
    return v.x * u.x + v.y * u.y


def rotate(p: Point, angle: float) -> Point:
    """
    Повернуть вектор p на угол angle
    """
    return Point(p.x * math.cos(angle) - p.y * math.sin(angle), p.y * math.cos(angle) + p.x * math.sin(angle))


def find_nearest_point(p: Point, points: list[Point], exclude: list[Point] = []) -> typing.Optional[Point]:
    """
    Найти ближайшую точку к p из облака points, игнорируя точки exclude
    """
    closest = None
    min_dist = 10e10
    for _, point in enumerate(points):
        if point in exclude:
            continue
        if dist(p, point) < min_dist:
            min_dist = dist(p, point)
            closest = point
    return closest


def find_nearest_robot(robo: Point, team: list[robot.Robot], avoid: list[int] = []) -> robot.Robot:
    """
    Найти ближайший робот из массива team к точке robot, игнорируя точки avoid
    """
    robo_id = -1
    min_dist = 10e10
    for i, player in enumerate(team):
        if i in avoid or not player.is_used():
            continue
        if dist(robo, player.get_pos()) < min_dist:
            min_dist = dist(robo, player.get_pos())
            robo_id = i
    return team[robo_id]


def wind_down_angle(angle: float) -> float:
    """
    Привести угол к диапазону [-pi, pi]
    """
    angle = angle % (2 * math.pi)
    if angle > math.pi:
        angle -= 2 * math.pi
    return angle


def closest_point_on_line(point1: Point, point2: Point, point: Point) -> Point:
    """
    Получить ближайшую к точке point току на линии point1-point2
    """
    line_vector = (point2.x - point1.x, point2.y - point1.y)
    line_length = dist(point1, point2)

    if line_length == 0:
        return point1

    line_direction = (line_vector[0] / line_length, line_vector[1] / line_length)

    point_vector = (point.x - point1.x, point.y - point1.y)
    dot_product = point_vector[0] * line_direction[0] + point_vector[1] * line_direction[1]

    if dot_product <= 0:
        return point1
    if dot_product >= line_length:
        return point2

    closest_point = Point(point1.x + line_direction[0] * dot_product, point1.y + line_direction[1] * dot_product)

    return closest_point


def point_on_line(robo: Point, point: Point, distance: float) -> Point:
    """
    Получить точку на линии robot-point,
    отстоящую от точки robot на расстояние distance
    """
    vec_arg = math.atan2(point.y - robo.y, point.x - robo.x)

    # Calculate the new point on the line at the specified distance from the robot
    new_x = robo.x + distance * math.cos(vec_arg)
    new_y = robo.y + distance * math.sin(vec_arg)
    return Point(new_x, new_y)


def lerp(p1: Point, p2: Point, t: float) -> Point:
    """
    Получить линейно интерполированное значение
    """
    return p1 * (1 - t) + p2 * t


def minmax(x: float, a: float, b: float) -> float:
    """
    Получить ближайшее к x число из диапазона [a, b]
    """
    return min(max(x, a), b)


def angle_to_point(point1: Point, point2: Point) -> float:
    """
    Получить угол вектора p = point2 - point1
    """
    return (point2 - point1).arg()


def sign(num: float) -> float:
    """
    Получить знак числа num (0 если num == 0)
    """
    if num == 0:
        return 0
    return num / abs(num)


def det(a: float, b: float, c: float, d: float) -> float:
    """
    Получить определитель матрицы:
    |a b|
    |c d|
    """
    return a * d - b * c


def line_intersect(m: BobLine, bots: list[BobLine]) -> typing.Optional[list[Point]]:
    """
    TODO написать доку
    """
    result = []
    for n in bots:
        mat_inv = det(m.A, m.B, n.A, n.B)
        res = Point(0, 0)
        if abs(mat_inv) < 1e-9:
            return None
        res.x = -det(m.C, m.B, n.C, n.B) / mat_inv
        res.y = -det(m.A, m.C, n.A, n.C) / mat_inv
        result.append(res)
    return result


def probability(inter: list[Point], bots: list[robot.Robot], pos: Point) -> float:
    """
    TODO написать доку
    """
    res = 1.0
    # print(len(inter), end = ' ')
    for i, intr in enumerate(inter):
        # koef = 1
        # print([inter[i].x, inter[i].y, bots[i].get_pos().x, bots[i].get_pos().y])
        tmp_res_x = dist(intr, bots[i].get_pos())
        tmp_res_y = math.sqrt(dist(pos, bots[i].get_pos()) ** 2 - tmp_res_x**2)
        ang = math.atan2(tmp_res_y, tmp_res_x)
        # abs(ang) < math.pi / 4
        if abs(ang) > math.pi / 2:
            continue
        # print(tmpRes)
        # if tmpResX < 0:
        #     koef = 0
        # elif tmpResX > const.ROBOT_R * 100 * 15:
        #     koef = 1
        # else:
        #     koef = tmpResX / (const.ROBOT_R * 100 * 15)
        # res *= (2 * abs(ang) / math.pi) * (dist(st, bots[i].get_pos()) / 54e6)
        res *= 1 / (2 * abs(ang) / math.pi)
    return res


def bot_position(pos: Point, vecx: float, vecy: float) -> Point:
    """
    TODO написать доку
    """
    modul = (vecx**2 + vecy**2) ** (0.5)
    vecx = (vecx / modul) * const.ROBOT_R * 1000 * 2
    vecy = (vecy / modul) * const.ROBOT_R * 1000 * 2
    return Point(pos.x - vecx, pos.y - vecy)


def shot_decision(pos: Point, end: list[Point], tobj: list:robot.Robot):
    """
    TODO написать доку
    """
    objs = tobj.copy()
    tmp_counter = 0
    for obj in range(len(objs)):
        if not objs[obj - tmp_counter].is_used():
            objs.pop(obj - tmp_counter)
            tmp_counter += 1
    # mx_shot_prob = 0
    shot_point = pos
    mx = 0
    # tmp_sum = Point(0, 0)
    # n = 0
    # print(st)
    # for bot in obj:
    #     # print([bot.get_pos().x, bot.get_pos().y], end = " ")
    #     plt.plot(bot.get_pos().x, bot.get_pos().y, 'bo')
    # t = np.arange(-4500*1.0, 1000*1.0, 10)
    for point in end:  # checkai
        A = -(point.y - pos.y)
        B = point.x - pos.x
        C = pos.x * (point.y - pos.y) - pos.y * (point.x - pos.x)
        tmp_line = BobLine(A, B, C)
        # plt.plot(t, (tmpLine.A*t + tmpLine.C)/tmpLine.B, 'g--')
        lines = []
        for bot in objs:
            tmp_c = -(B * bot.get_pos().x - A * bot.get_pos().y)
            line2 = BobLine(B, -A, tmp_c)
            lines.append(line2)
        inter = line_intersect(tmp_line, lines)
        # plt.plot(inter[0].x, inter[0].y, 'bx')
        # plt.plot(inter[1].x, inter[1].y, 'gx')
        tmp_prob = probability(inter, objs, pos)
        # print(tmp_prob, end = " ")
        if tmp_prob > mx:
            mx = tmp_prob
            shot_point = bot_position(pos, point.x - pos.x, point.y - pos.y)
            point_res = point
        # if tmp_prob > mx:
        #     mx = tmp_prob
        #     shot_point = botPosition(st, point.x - st.x, point.y - st.y)
        #     point_res = point
        #     n = 1
        #     sum = point
        # elif tmp_prob == mx:
        #     sum += point
        #     n += 1
        # else:
        #     point_res = sum / n
        #     shot_point = botPosition(st, point_res.x - st.x, point_res.y - st.y)
        #     sum = Point(0, 0)
        #     n = 0
    # plt.plot(t, -(point_res.A*t + point_res.C)/point_res.B, 'r-')
    # plt.plot(shot_point.x, shot_point.y, 'r^')
    # plt.axis('equal')
    # plt.grid(True)
    # plt.show()
    return shot_point, mx, point_res


def in_place(point, end, epsilon):
    """
    Проверить, находится ли точка st в радиусе epsilon около end
    """
    return (point - end).mag() < epsilon


def is_in_list(arr, x):
    """
    Узнать, есть ли элемент x в списке arr
    """
    return x in arr
