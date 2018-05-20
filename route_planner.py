import sys
from vector import Vector
from operator import itemgetter


class WayPoint:
    def __init__(self, x=0.0, y=0.0, dist=-1.0):
        self.pos = Vector(x, y)
        self.distance = dist
        self.neighbors = []  # List of WayEdge instances
        # TODO : Store additional user attribute
        # TODO : Add runtime availability flag

    @property
    def reachable(self):
        return len(self.neighbors) > 0


class WayPointEdge:
    def __init__(self, source, target):
        self.source = source  # WayPoint
        self.target = target  # WayPoint
        self.dist = Vector.distance(self.source.pos, self.target.pos)
        # TODO : Store additional user attribute and cost
        # TODO : Add runtime availability flag

    @property
    def distance(self):
        return self.dist  # TODO : cache/dirty flag/recalculation

    @property
    def s(self):
        return self.source

    @property
    def t(self):
        return self.target

    def opposite(self, wp):
        if self.source == wp:
            return self.target
        elif self.target == wp:
            return self.source
        else:
            return None


class Route:
    """Node of doubly-linked list for representing one result."""
    def __init__(self, wp=None):
        self.way_point = wp
        self.prev_route = None
        self.next_route = None
        self.distance_actual = 0.0
        self.distance_heuristic = 0.0
        self.distance_estimated = 0.0


class RoutePlanner:
    def __init__(self, wps):
        self.way_points = wps

    # TODO : Implement hierarchy search
    def find_nearest_way_point(self, pos):
        min_dist = sys.float_info.max
        nearest_way_point = None
        for wp in self.way_points:
            if wp.reachable == False:
                continue
            dist = Vector.distance(wp.pos, pos)
            if dist < min_dist:
                nearest_way_point = wp
                min_dist = dist
        return nearest_way_point

    def plan(self, start_pos, goal_pos):
        start_wp = self.find_nearest_way_point(start_pos)
        goal_wp = self.find_nearest_way_point(goal_pos)
        if start_wp == goal_wp:
            return None

        route_plan = []
        current_route = Route(start_wp)

        frontier_list = [current_route]
        explored_list = []

        while frontier_list:
            frontier_list.sort(key=lambda n: n.distance_estimated)
            current_route = frontier_list.pop(0)
            if current_route.way_point == goal_wp:
                break
            dist_so_far = current_route.distance_actual
            explored_list.append(current_route)
            for edge in current_route.way_point.neighbors:
                # TODO : Check edge/wai_point availability here
                wp_neighbor = edge.opposite(current_route.way_point)
                if not wp_neighbor.reachable:
                    continue
                if any(x for x in explored_list if x.way_point == wp_neighbor):
                    continue
                if any(x for x in frontier_list if x.way_point == wp_neighbor):
                    n = [x for x in frontier_list if x.way_point == wp_neighbor][0]
                    if n.distance_actual <= current_route.distance_actual + edge.distance:
                        continue

                new_node = Route(wp_neighbor)
                new_node.prev_route = current_route
                current_route.next = new_node
                new_node.distance_actual = dist_so_far + edge.distance  #  Vector.distance(wp_neighbor.pos, current_route.way_point.pos)
                new_node.distance_heuristic = Vector.distance(wp_neighbor.pos, goal_wp.pos)
                new_node.distance_estimated = new_node.distance_actual + new_node.distance_heuristic  # TODO : Implement additional cost callback
                frontier_list.append(new_node)
        if current_route.way_point == goal_wp:
            while current_route.prev_route:
                route_plan.append(current_route.way_point.pos)
                current_route = current_route.prev_route
            route_plan.append(start_wp.pos)
            route_plan.reverse()

        return route_plan
