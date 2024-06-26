import matplotlib.pyplot as plt
import numpy as  np
from path_helper import *

from queue import PriorityQueue
import math


def calc_cost(cell_1, cell_2):
    """
    Calculate distance between two cells - used for travel and heuristic cost
    :param cell_1: List[int] 1 x 2 - x y position
    :param cell_2: List[int] 1 x 2 - x y position
    :return: euclidean distance between 2 locations
    """
    return math.dist(cell_1, cell_2)

class Node:
    def __init__(self,xy,xy_ind,par_ind,travel_cost,goal):
        """
        Create node
        :param xy: List[int] 1 x 2 - x y position
        :param xy_ind: int - index in grid
        :param par_ind: int - index of parent node
        :param travel_cost: int - cost to get from start position to this position
        :param goal: List[int] 1 x 2 - x y position of goal - used to cal heuristic cost
        """

        self.xy = xy
        self.ind = xy_ind
        self.par_ind = par_ind
        self.travel_cost = travel_cost
        self.heuristic_cost = calc_cost(xy, goal)

    def __eq__(self, other):
        return self.ind == other.ind

    def __ne__(self, other):
        return self.ind != other.ind

    def __lt__(self, other):
        return self.ind < other.ind

    def __le__(self, other):
        return self.ind <= other.ind

    def __gt__(self, other):
        return self.ind > other.ind

    def __ge__(self, other):
        return self.ind >= other.ind

    def __repr__(self):
        return "{}".format(self.xy)
    



def compute_path_astar(grid, start, goal, obs):
    """
    Compute path using a star
    :param grid: List[int] 1 x 2 - size of grid map
    :param start: List[int] 1 x 2 - start pose
    :param goal: List[int] 1 x 2 - goal pose
    :param obs: List[int] n x 2 - obstacle pose
    :return: path List[int] n x 2 - path from start to goal pose
        nodes_explored List[int] n x 2 - nodes explored while finding path
    """

    nodes_explored = {}
    goal_reached = False

    pq = PriorityQueue()
    start_ind = grid_to_index(grid, start)
    start_node = Node(start, start_ind, -1, 0, goal)

    goal_ind = grid_to_index(grid, goal)
    goal_node = Node(goal, goal_ind, -1, 0, goal)

    pq.put([0, start_node])

    while (not pq.empty()) and (goal_reached is not True):

        # Get node to explore with least cost (cost to reach current node & reach goal)
        _, parent_node = pq.get()

        # Since our priority queue has multiple copies of same node, check to see if explored before
        if parent_node.ind not in nodes_explored:

            list_child = compute_child_nodes(parent_node.xy)

            for child_xy in list_child:

                travel_cost = calc_cost(child_xy, parent_node.xy) + parent_node.travel_cost

                if child_xy == goal:
                    goal_reached = True
                    goal_node.par_ind = parent_node.ind
                    goal_node.travel_cost = travel_cost
                    print("Goal reached")
                    break

                # Valid move - within grid & not an obstacle
                if check_valid_move(child_xy, grid, obs):

                    child_ind = grid_to_index(grid, child_xy)

                    if child_ind not in nodes_explored:

                        child_node = Node(child_xy, child_ind, parent_node.ind, travel_cost, goal)
                        total_cost = child_node.travel_cost + child_node.heuristic_cost
                        pq.put([total_cost, child_node])

            nodes_explored[parent_node.ind] = parent_node

    path = construct_path_nodes(nodes_explored, goal_node)

    return path, nodes_explored

def main():
    grid_size=(10,10)
    start_pose=(8,2)
    goal_pose=[0,5]
    obstacles=[
        [1, 3],
        [0, 4],
        [3, 4],
        [7, 5]
    ]
    path_bfs, nodes_bfs = compute_path_astar(grid_size, start_pose, goal_pose, obstacles)
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

