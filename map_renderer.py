import sdl2
from route_planner import WayPoint, Route


def render_waypoints(renderer, waypoints):
    for wp in waypoints:
        sdl2.aacircleColor(renderer, int(wp.pos.x), int(wp.pos.y), 5, 0xFFC0C0C0)

    # TODO : Fix overdrawing
    for wp in waypoints:
        for we in wp.neighbors:
            sdl2.aalineColor(renderer, int(we.s.pos.x), int(we.s.pos.y), int(we.t.pos.x), int(we.t.pos.y), 0xFFFFFFFF)


def render_route_plan(renderer, start_pos, goal_pos, route_plan):
    # Start Node
    sdl2.filledCircleColor(renderer, int(start_pos.x), int(start_pos.y), 8, 0xFFFF0000)
    # Goal Node
    sdl2.filledCircleColor(renderer, int(goal_pos.x), int(goal_pos.y), 8, 0xFF0000FF)

    # Route Node
    if not route_plan:
        return

    for r in route_plan:
        sdl2.filledCircleColor(renderer, int(r.x), int(r.y), 4, 0xFF00FF00)

    # Route Edge
    if route_plan:
        sdl2.aalineColor(renderer, int(start_pos.x), int(start_pos.y), int(route_plan[0].x), int(route_plan[0].y), 0xFF00FF00)
        for i in range(0, len(route_plan) - 1):
            sdl2.aalineColor(renderer, int(route_plan[i].x), int(route_plan[i].y), int(route_plan[i+1].x), int(route_plan[i+1].y), 0xFF00FF00)
        sdl2.aalineColor(renderer, int(route_plan[-1].x), int(route_plan[-1].y), int(goal_pos.x), int(goal_pos.y), 0xFF00FF00)

