import sys
from pathlib import Path
import ctypes
import ctypes.util

import sdl2
from vector import Vector
from route_planner import WayPoint, WayPointEdge, RoutePlanner
import map_renderer

# import sys, os, threading, time
# sys.path.append(os.pardir)


WINDOW_W = 800
WINDOW_H = 450

def read_map(roadmap_input):
    csv_lines = roadmap_input.readlines()
    cols = csv_lines.pop(0).split(",")
    nodes_count = int(cols[0].strip())
    edges_count = int(cols[1].strip())
    obstacles_count = int(cols[2].strip())
    # print(nodes_count, edges_count, obstacles_count)

    nodes = []
    nodes_index_map = {}
    edges = []

    for i in range(0, nodes_count):
        id, x, y = csv_lines.pop(0).split(",")
        x = int(x) * 25
        y = int(y) * 18
        nodes.append(Vector(x, y))
        nodes_index_map[id] = i

    for i in range(0, edges_count):
        cost, node0_id, node1_id = csv_lines.pop(0).split(",")
        edges.append([nodes_index_map[node0_id.strip()], nodes_index_map[node1_id.strip()]])

    obstacles = []
    for i in range(0, obstacles_count):
        obstacles.append(csv_lines.pop(0).strip())
    for ob in obstacles:
        edges = [x for x in edges if nodes_index_map[ob] not in x]

    return nodes, edges

def main():

    nodes = None
    edges = None

    if len(sys.argv) >= 2 and Path(sys.argv[1]).exists():
        map_file = Path(sys.argv[1])
        with map_file.open() as roadmap_input:
            nodes, edges = read_map(roadmap_input)
            # print(nodes, edges)
    else:
        nodes = [Vector(100.0, 100.0), Vector(200.0, 200.0), Vector(300.0, 100.0), Vector(400, 200)]
        edges = [[0, 1], [1, 2], [2, 0], [1, 3], [2, 3]]

    wps = []
    for n in nodes:
        wps.append(WayPoint(n.x, n.y))

    for e in edges:
        edge = WayPointEdge(wps[e[0]], wps[e[1]])
        wps[e[0]].neighbors.append(edge)
        wps[e[1]].neighbors.append(edge)
        # print(Vector.distance(wps[e[0]].pos, wps[e[1]].pos))

    rp = RoutePlanner(wps)
    start = Vector(0.0, 0.0)
    goal = Vector(500.0, 200.0)
    route_plan = rp.plan(start, goal)

    sdl2.sdl2_load(ctypes.util.find_library('SDL2'),  # '/usr/local/lib/libSDL2.dylib'
                   gfx_libpath=ctypes.util.find_library('SDL2_gfx')
                   )
    sdl2.SDL_Init(sdl2.SDL_INIT_EVERYTHING)

    window = sdl2.SDL_CreateWindow(b"A* demonstration", 0, 0, WINDOW_W, WINDOW_H, sdl2.SDL_WINDOW_OPENGL)

    renderer = sdl2.SDL_CreateRenderer(window, -1, 0)

    mouse_x = ctypes.c_int()
    mouse_y = ctypes.c_int()

    fps_delay = 100
    event = sdl2.SDL_Event()
    done = False
    while not done:
        while sdl2.SDL_PollEvent(ctypes.byref(event)) != 0:
            # 'type' and 'timestamp' are common members for all SDL Event structs.
            event_type = event.common.type
            # event_timestamp = event.common.timestamp
            # print("Event : type=0x%s, timestamp=%s" % (event_type, event_timestamp) )

            if event_type == sdl2.SDL_KEYDOWN:
                if event.key.keysym.sym == sdl2.SDLK_ESCAPE:
                    done = True
                if event.key.keysym.sym == sdl2.SDLK_SPACE:
                    route_plan = rp.plan(start, goal)

        sdl2.SDL_SetRenderDrawColor(renderer, 0xA0, 0xA0, 0xA0, 0xFF)
        sdl2.SDL_RenderClear(renderer)

        mouse_state = sdl2.SDL_GetMouseState(ctypes.byref(mouse_x), ctypes.byref(mouse_y))
        if mouse_state == (1 << (sdl2.SDL_BUTTON_LEFT - 1)):
            prev_x = start.x
            prev_y = start.y
            start.x = float(mouse_x.value)
            start.y = float(mouse_y.value)
            if start.x != prev_x or start.y != prev_y:
                route_plan = rp.plan(start, goal)
        elif mouse_state == (1 << (sdl2.SDL_BUTTON_RIGHT - 1)):
            prev_x = goal.x
            prev_y = goal.y
            goal.x = float(mouse_x.value)
            goal.y = float(mouse_y.value)
            if goal.x != prev_x or goal.y != prev_y:
                route_plan = rp.plan(start, goal)

        map_renderer.render_waypoints(renderer, wps)
        map_renderer.render_route_plan(renderer, start, goal, route_plan)
        sdl2.SDL_RenderPresent(renderer)

        sdl2.SDL_Delay(fps_delay)

    sdl2.SDL_DestroyRenderer(renderer)
    sdl2.SDL_DestroyWindow(window)
    sdl2.SDL_Quit()


if __name__ == '__main__':
    main()
