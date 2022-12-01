import numpy as np
import matplotlib.pyplot as plt
import math 

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


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = node(None, end)
    end_node.g = end_node.h = end_node.f = 0

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
        for i in range(0,8):
            if i==0:
                if (current_node.position[1]-1< (len(maze))):
                    if maze[current_node.position[0]][current_node.position[1]-1]!=1:
                        new_node_postion = (current_node.position[0], current_node.position[1]-1)
                new_node = node(current_node, new_node_postion)
                children.append((new_node, i, maze[current_node.position[0]][current_node.position[1]-1]))
            
            if i==1:
                if (current_node.position[1]+1>=0):
                    if maze[current_node.position[0]][current_node.position[1]+1]!=1:
                        new_node_postion = (current_node.position[0], current_node.position[1]+1)
                new_node = node(current_node, new_node_postion)
                children.append((new_node, i, maze[current_node.position[0]][current_node.position[1]+1]))

            if i==2:
                if (current_node.position[0]-1>=0):
                    if maze[current_node.position[0]-1][current_node.position[1]]!=1:
                        new_node_postion = (current_node.position[0]-1, current_node.position[1])
                new_node = node(current_node, new_node_postion)
                children.append((new_node, i, maze[current_node.position[0]-1][current_node.position[1]]))
            
            if i==3:
                if (current_node.position[0]+1<(len(maze[0]))):
                    if maze[current_node.position[0]+1][current_node.position[1]]!=1:
                        new_node_postion = (current_node.position[0]+1, current_node.position[1])
                new_node = node(current_node, new_node_postion)
                children.append((new_node, i, maze[current_node.position[0]+1][current_node.position[1]]))

            if i==4:
                if (current_node.position[0]-1>=0 and current_node.position[1]-1< (len(maze))):
                    if maze[current_node.position[0]-1][current_node.position[1]-1]!=1:
                        new_node_postion = (current_node.position[0]-1, current_node.position[1]-1)
                new_node = node(current_node, new_node_postion)
                children.append((new_node, i, maze[current_node.position[0]-1][current_node.position[1]-1]))

            if i==5:
                if (current_node.position[0]-1>=0 and current_node.position[1]+1>=0):
                    if maze[current_node.position[0]-1][current_node.position[1]+1]!=1:
                        new_node_postion = (current_node.position[0]-1, current_node.position[1]+1)
                new_node = node(current_node, new_node_postion)
                children.append((new_node, i, maze[current_node.position[0]-1][current_node.position[1]+1]))
            if i==6:
                if (current_node.position[0]+1<(len(maze[0])) and current_node.position[1]-1< (len(maze))):
                    if maze[current_node.position[0]+1][current_node.position[1]-1]!=1:
                        new_node_postion = (current_node.position[0]+1, current_node.position[1]-1)
                new_node = node(current_node, new_node_postion)
                children.append((new_node, i, maze[current_node.position[0]+1][current_node.position[1]-1]))

            if i==7:
                if (current_node.position[0]+1<=(len(maze[0])) and current_node.position[1]+1>=0):
                    if maze[current_node.position[0]+1][current_node.position[1]+1]!=1:
                        new_node_postion = (current_node.position[0]+1, current_node.position[1]+1)
                new_node = node(current_node, new_node_postion)
                children.append((new_node, i, maze[current_node.position[0]+1][current_node.position[1]+1]))

        # Loop through children to update the costs
        for child in children:
            # Child is on the closed list
            for closed_child in closed_list:
                if child[0] == closed_child:
                    break
            else:
                if child[1]==4 or child[1]==5 or child[1]==6 or child[1]==7:
                # Create the f, g, and h values, replace the 0s with appropriate formulations of the costs
                    child[0].g = current_node.g + math.sqrt(2)+child[2]
                else:
                    child[0].g = current_node.g + 1
                child[0].h = ((child[0].position[0] - end_node.position[0]) ** 2) + ((child[0].position[1] - end_node.position[1]) ** 2)
                child[0].f = child[0].g + child[0].h + child[2]

                # Complete here code to check whether to add a child to the open list
                for open_node in open_list:
                    if child[0] == open_node and child[0].g > open_node.g:
                        continue
                open_list.append(child[0])


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
