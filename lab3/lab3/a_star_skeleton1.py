import numpy as np
import matplotlib.pyplot as plt
import math 
import sys

class node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def check_in_bounds(width, height, px_x, px_y):
    if (px_x < 0 or px_x > width-1):
        return False
    if (px_y < 0 or px_y > height-1):
        return False
    return True

def get_distance(node1: node, node2: node):
    pos1 = node1.position
    pos2 = node2.position
    return math.sqrt(pow(pos1[0]-pos2[0],2)+ pow(pos1[1]-pos2[1],2))


def astar(maze, start, end, ogm_threshold):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    # Create start and end node
    start_node = node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = node(None, end)
    end_node.g = end_node.h = end_node.f = 0
    # Save width and height of image
    height, width = maze.shape
    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end node
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            # Get lowest cost
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal, you can also implement what should happen if there is no possible path
        if current_node == end_node:
            
            # TODO: Check if returns shortest path found
            path = []
            current = current_node
            while current:
                path.append(current.position)
                current = current.parent
            return path

        # Complete here code to generate children, which are the neighboring nodes. You should use 4 or 8 points connectivity for a grid.
        children = []
        distance = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))
        for d_x, d_y in distance:
            child_x = current_node.position[0]+d_x
            child_y = current_node.position[1]+d_y
            # Check if child is within range of map
            if (check_in_bounds(width, height, child_x, child_y)):
                # Check if the maze location is occupied
                if maze[child_y][child_x] < ogm_threshold:
                    pose = (child_x, child_y)
                    new_node = node(current_node, pose)
                    children.append(new_node)
       
        for child in children:
            if child not in closed_list:
                # Calcuate distance from current node to child
                distance = get_distance(current_node, child)
                child.g = current_node.g + distance
                child.h = get_distance(child, end_node)
                child.f = child.g + child.h
                if child in open_list:
                    # find index of same child node
                    index = open_list.index(child)
                    existing_child = open_list[index]
                    if existing_child.f > child.f:
                        # Remove existing child, and append this child instead
                        open_list.pop(index)
                        open_list.append(child)
                else:
                    open_list.append(child)
                    
def main():

    # Load your maze here
    maze = []
    
    # This is an example maze you can use for testing, replace it with loading the actual map
    maze = [[0,   0,   0,   0,   1,   0, 0, 0, 0, 0],
            [0, 0.8,   1,   0,   1,   0, 0, 0, 0, 0],
            [0, 0.9,   1,   0,   1,   0, 1, 0, 0, 0],
            [0,   1,   0,   0,   1,   0, 1, 0, 0, 0],
            [0,   1,   0,   0,   1,   0, 0, 0, 0, 0],
            [0,   0,   0, 0.9,   0,   1, 1, 1, 1, 1],
            [0,   0, 0.9,   1,   1, 0.7, 0, 0, 0, 0],
            [0,   0,   0,   1,   0,   1, 0, 0, 0, 0],
            [0,   0,   0,   0, 0.9,   1, 0, 0, 0, 0],
            [0,   0,   0,   0,   0,   1, 0, 0, 0, 0]]

    
    # Define here your start and end points
    start = (0, 0)
    end = (6,6)
    
    # Compute the path with your implementation of Astar
    path = np.asarray(astar(maze, start, end), dtype=np.float)
    maze_plot=np.transpose(np.nonzero(maze))
    print(path[::-1])
    print (type(path))

    plt.plot(maze_plot[:,0], maze_plot[:,1], 'o')
    
    if not np.any(path): # If path is empty, will be NaN, check if path is NaN
        print("No path found")
    else:
        print ("dfd")
        plt.plot(path[:,0], path[:,1])
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()
