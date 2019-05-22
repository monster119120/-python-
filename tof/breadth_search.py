from implementation import *
import numpy as np
qizi = [[1,   0,  0,  0,  0,  0,  0,  0],
        [0,   1,  0,  0,  0,  0,  0,  0],
        [0,   0,  1,  0,  0,  0,  0,  0],
        [0,   0,  0,  1,  0,  0,  0,  0],
        [0,   0,  0,  0,  1,  0,  0,  0],
        [0,   0,  0,  0,  0,  1,  0,  0],
        [0,   0,  0,  0,  0,  0,  1,  1],
        [0,   0,  0,  0,  0,  0,  0,  0]]

def breadth_first_search_3(graph, start, goal):
    frontier = Queue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current

    return came_from

# g = SquareGrid(30, 15)
# g.walls = DIAGRAM1_WALLS
# parents = breadth_first_search_3(g, (8, 7), (17, 2))

# temp = (17,2)
# route = []
# print(temp)
# print(route)

def mya_star(qizi = qizi,start = (),goal = ()):
    g = SquareGrid(8,8)
    g.walls =np.argwhere(np.array(qizi) == 1).tolist()
    parents = breadth_first_search_3(g, start, goal)
    temproute = []
    temp = start
    while (1):
        if temp == None:
            break
        temproute.append(temp)
        temp = parents[temp]
    return temproute[::-1]