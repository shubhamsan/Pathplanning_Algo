import matplotlib.pyplot as plt
import numpy as  np
from path_helper import *


def compute_path_bfs(grid,start,goal,obst):
    """Compute path using breadth first search
    :param grid: list (2) - x y grid size
    :param start: list (2) - x y pose for start position
    :param goal: list (2) - x y pose for goal position
    :param obs: list(n, 2) - x y pose of multiple obstacles
    :return: path - list (n, 2) - path for robot from start to goal
    """
    nodes_explored = {}
    queue = [start]
    goal_reached=False
    parents = {grid_to_index(grid,start):None}

    # Search until we find the goal OR until we ran out of nodes to explore
    while (not goal_reached) and (len(queue)>0):

    # Look at a node from queue and explore it
        parent_node =queue.pop(-1)
        print(parent_node)
        
    # List of moves from this position
        child_nodes = compute_child_nodes(parent_node)

    # for each move
        for child in child_nodes:
            print("Index:", grid_to_index(grid,child),"child",child,goal)
        # move leads to goal?
            if child == list(goal):
                goal_reached=True
                parents[grid_to_index(grid,child)]=parent_node
                print("Goal reached!!!")
                break

        # (1) within grid size?
        # (2) Obstacle?
        # (3) Visited before?
            if check_valid_move(child,grid,obst):
                if grid_to_index(grid,child) not in nodes_explored:
                    if child not in queue:
                        # Add node to explore in future.

                        queue.append(child)
                        parents[grid_to_index(grid,child)]=parent_node
              

    # mark the node as explored
        nodes_explored[grid_to_index(grid,parent_node)]=None
        # print(nodes_explored)
        path =[]
        if goal_reached:
            path = construct_path(grid,parents,goal)

    return path, nodes_explored
def main():
    grid_size=(10,10)
    start_pose=(8,2)
    goal_pose=(9,0)
    obstacles=[
        [1, 3],
        [0, 4],
        [3, 4],
        [7, 5]
    ]
    path_bfs, nodes_bfs = compute_path_bfs(grid_size, start_pose, goal_pose, obstacles)
    print("Path: {}".format(path_bfs))
    print("Length of path: {}".format(len(path_bfs)))
    print("Number of nodes explored: {}".format(len(nodes_bfs)))
    # for i in range(0,5):
    #     for j in range(0,5):
    #         print("Index:", grid_to_index(grid_size,(i,j)),"rc: ",index_to_grid(grid_size,grid_to_index(grid_size,(i,j))))

    animation=True
    plot_planner(grid_size,start_pose,goal_pose,obstacles,path_bfs,nodes_bfs,animation)

if __name__ =="__main__":
    main()

