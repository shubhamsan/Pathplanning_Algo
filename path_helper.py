
from matplotlib import pyplot as plt
import numpy as np
from datetime import datetime
import random


def plot_planner(grid_size,start_pose,goal_pose,obstacles,path,nodes_explored,animation):
    """
    Plot the grid map. Show start and goal post, path, obstacles, etc.
    :param grid_size: list(2) - grid size x & y
    :param start_pose: list(2) - x & y pose of start position
    :param goal_pose: list (2) - x & y pose of goal position
    :param obstacles: list (n, 2) - x & y pose of multiple obstacles
    :param path: list (n, 2) - x & y pose of each cell in the path
    :param nodes_explored: dict (n) - index of nodes explored
    :param animation: bool - to show animation in plot or not
    :return:
    """
    fig,ax= plt.subplots()
    ax.set(xlim =(0,grid_size[0]),xticks = np.arange(1,grid_size[1]),
           ylim = (0,grid_size[1]),yticks = np.arange(1,grid_size[1]))
    plt.grid()
    #plot start and end pose
    plt.fill_between([start_pose[1],start_pose[1]+1],start_pose[0],start_pose[0]+1,edgecolor = "green",color="green",linewidth=5)
    plt.fill_between([goal_pose[1], goal_pose[1] + 1], goal_pose[0], goal_pose[0] + 1,edgecolor='red',color="red", linewidth=5)
    #plot the obstacles
    for each_obj in obstacles:
        plt.fill_between([each_obj[1],each_obj[1]+1],each_obj[0],each_obj[0]+1,edgecolor = "black",color="black",linewidth=5)
    # Plot nodes explored
    for each_node in nodes_explored:
        x, y = index_to_grid(grid_size, each_node)
        plt.fill_between([y, y+1], x, x+1, color='yellow')
        if animation:
            plt.pause(0.5)

    for each_pa in path:
        plt.fill_between([each_pa[1], each_pa[1] + 1], each_pa[0], each_pa[0] + 1, color='blue')
        if animation:
            plt.pause(0.5)
    plt.show()

def index_to_grid(grid, ind):
    """
    convert index number in to grid position of x y
    :param grid: list(2) - x y max limit of grid
    :param ind: (int) index value of node in grid
    :return: node list (2) x y pose in grid
    """
    r = ind // grid[0]
    c = ind % grid[1]
    return [r, c]

def grid_to_index(grid,node):
    """
    Convert grid x y pose to index value
    :param grid: list(2) - x y max limit of grid
    :param node: list(2) - x y pose
    :return: index (int) - index of node in grid
    """
    return (node[0] *grid[0]) +node[1]

def compute_child_nodes(parent_node):
    """
    A list of possible moves for robot. Up, Down, Left, Right and diagonals.
    :param parent_node: list(2) - x y pose of current node
    :return: list(n, 2) - x y pose of different moves.
    """
    px,py = parent_node
    chid_node = [
        [px+1,py+1],#up-right
        [px+1,py-1],#bottom right

        [px+1,py],  #right
        [px-1,py],  #left

        [px,py+1],  #top
        [px,py-1],  #bottom

        [px-1,py+1],#left top
        [px-1,py-1],#left bottom

    ]
    return chid_node



def check_valid_move(node, grid, obs):
    """
    Check if move is valid - no obstacle and not outside grid
    :param node: list(2) - x y of current pose
    :param grid: list(2) - x y limit of grid size
    :param obs: list(n, 2) - x y pose of multiple obstacles
    :return: True/False to indicate valid move
    """
    valid_move = True
    if node in obs:
        valid_move=False
    elif node[0]<=0 or node[1]<=0:
        valid_move =False
    elif node[0]>=grid[0] or node[1]>=grid[1]:
        valid_move =False
    return valid_move



def construct_path(grid, parents, goal):
    """
    Compute path in our searched map
    :param grid: list(2) - x y max limit of grid
    :param parents: dict(n) - list of parent index and child node in x y
    :param goal: list(2) - x y pose of goal
    :return: list(n, 2) - path from start to goal in x y
    """
    path = []
    current_node = goal 
    while current_node:
        path.append(current_node)
        parent_node= parents[grid_to_index(grid,current_node)]
        current_node=parent_node
    return path[::-1]


def construct_path_nodes(nodes_explored, goal_node):
    path = []
    cur_node = goal_node
    while cur_node.par_ind != -1:
        path.append(cur_node.xy)
        parent_ind = cur_node.par_ind
        cur_node = nodes_explored[parent_ind]

    path.append(cur_node.xy)

    return path[::-1]


