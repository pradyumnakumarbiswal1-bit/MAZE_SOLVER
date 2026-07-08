from pyamaze import maze, agent, COLOR, textLabel
from queue import PriorityQueue
import time


def h(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return abs(x1 - x2) + abs(y1 - y2)


def animate_path(m, agent_obj, cells, delay=0.08):
    for cell in cells:
        agent_obj.position = cell
        m._canvas.update()
        time.sleep(delay)


def astar(m):
    start = (m.rows, m.cols)
    goal = (1, 1)

    g_score = {cell: float('inf') for cell in m.grid}
    g_score[start] = 0

    f_score = {cell: float('inf') for cell in m.grid}
    f_score[start] = h(start, goal)

    open_queue = PriorityQueue()
    open_queue.put((f_score[start], h(start, goal), start))

    parent = {}
    search_path = []
    visited = set()

    while not open_queue.empty():
        currCell = open_queue.get()[2]

        if currCell in visited:
            continue
        visited.add(currCell)
        search_path.append(currCell)

        if currCell == goal:
            break

        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])

                temp_g_score = g_score[currCell] + 1
                temp_f_score = temp_g_score + h(childCell, goal)

                if temp_f_score < f_score[childCell]:
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_f_score
                    open_queue.put((temp_f_score, h(childCell, goal), childCell))
                    parent[childCell] = currCell

    reverse_path_cells = [goal]
    cell = goal
    while cell != start:
        cell = parent[cell]
        reverse_path_cells.append(cell)

    forward_path = {}
    cell = goal
    while cell != start:
        forward_path[parent[cell]] = cell
        cell = parent[cell]

    forward_path_cells = [start]
    cell = start
    while cell != goal:
        cell = forward_path[cell]
        forward_path_cells.append(cell)

    return search_path, reverse_path_cells, forward_path_cells, forward_path


if __name__ == '__main__':
    m = maze(10, 10)
    m.CreateMaze(loopPercent=20)

    search_path, reverse_path_cells, forward_path_cells, forward_path = astar(m)

    search_agent = agent(m, footprints=True, color=COLOR.yellow, filled=False)
    reverse_agent = agent(m, footprints=True, color='cyan', filled=False)
    reverse_agent_2 = agent(m, footprints=True, color='green', filled=False)
    forward_agent = agent(m, footprints=True, color=COLOR.blue, filled=True)

    animate_path(m, search_agent, search_path, delay=0.08)
    animate_path(m, reverse_agent, reverse_path_cells, delay=0.08)
    animate_path(m, reverse_agent_2, reverse_path_cells, delay=0.08)
    animate_path(m, forward_agent, forward_path_cells, delay=0.08)

    # Final forward path shown last, after the reverse path animations
    forward_agent.position = forward_path_cells[0]
    m._canvas.update()

    m.tracePath({forward_agent: forward_path})
    textLabel(m, 'A Star path length', len(forward_path_cells))
    m.run()