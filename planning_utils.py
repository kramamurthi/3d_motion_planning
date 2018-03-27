from enum import Enum
from queue import PriorityQueue
import numpy as np
import networkx as nx
from bresenham import bresenham
from scipy.spatial import Voronoi, voronoi_plot_2d
from matplotlib import pyplot as plt
import pickle

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


def safe_grid(grid,x,y):
    if x < 0 or x >= grid.shape[0] or y < 0 or y >= grid.shape[1]:
        return 1
    else:
        return grid[x][y]
    
def get_edges_from_cache_or_calculate(graph, grid, da, sd):
    # Load edges from pickle cache if available

    # The cache is keyed by drone_altitude, and safety_distance
    pickle_filename = './edges_cache_{}_{}.p'.format(da, sd)
    try:
        with open(pickle_filename,'rb') as f:
            edges = pickle.load(f)
            loaded_pickle = True
            print('pickle file found will fetch edges')

    except FileNotFoundError:
        print('pickle file not found will calculate edges')
        loaded_pickle = False
        
    if loaded_pickle:
        return edges

    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]

        x1 = np.int(p1[0])
        y1 = np.int(p1[1])
        x2 = np.int(p2[0])
        y2 = np.int(p2[1])
        
        cells = list(bresenham(x1,y1,x2,y2))   
        # Check for obstacles
        in_collision = np.any([safe_grid(grid,x,y) for x,y in cells])
        if not in_collision:
            edges.append((p1, p2))


    # Save to pickle
    print('pickle file saved for edges')
    pickle.dump(edges, open(pickle_filename, 'wb'))
    return edges

        
def create_grid_and_edges(data, drone_altitude, safety_distance, use_cache=True):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    
    # Define a list to hold Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(north - d_north - safety_distance - north_min_center),
                int(north + d_north + safety_distance - north_min_center),
                int(east - d_east - safety_distance - east_min_center),
                int(east + d_east + safety_distance - east_min_center),
            ]
            grid[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3]] = 1
            
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])


    # location of obstacle centres
    graph = Voronoi(points)
    edges = get_edges_from_cache_or_calculate(graph, grid, drone_altitude, safety_distance)
        
    return grid, int(north_min), int(east_min), edges

# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    return valid_actions


def a_star(grid, h, start, goal):
    """
    Given a grid and heuristic function returns
    the lowest cost path from start to goal.
    """

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            # Get the new vertexes connected to the current vertex
            for a in valid_actions(grid, current_node):
                next_node = (current_node[0] + a.delta[0], current_node[1] + a.delta[1])
                new_cost = current_cost + a.cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node, a)

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

def extract_graph_start_goal(edges, grid_start, grid_goal):
    G = nx.Graph()
    for e in edges:
        p1, p2 = e 
        dist = np.linalg.norm(np.array(p2) - np.array(p1))
        G.add_edge(tuple(p1), tuple(p2), weight=dist)

    graph_nodes = np.array(G.nodes())

    idx_start = np.argmin([np.linalg.norm(t) for t in graph_nodes-grid_start])
    idx_goal = np.argmin([np.linalg.norm(t) for t in graph_nodes-grid_goal])

    start_node = tuple(graph_nodes[idx_start])
    goal_node = tuple(graph_nodes[idx_goal])

    print(start_node, goal_node)  
    pickle.dump(G, open('graph.p','wb'))
    return G, start_node, goal_node  
    
def a_star_g(graph, heur, start, goal):


    # Handle the case of start and goal being the same
    if start == goal:
        print('Going nowhere')
        return [], 0.0
    
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
            
        else:
            for next_node in graph.adj[current_node]:
                cost = graph[current_node][next_node]['weight']
                new_cost = current_cost + cost + heur(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    branch[next_node] = (new_cost, current_node, next_node)
             
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][2])
            
    return path[::-1], path_cost

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


def prune_path(path, grid, use_bresenham=False):
    
    if path == []:
        return []

    if use_bresenham:
        pruned_path = []
        # Start with first point and see how farthest of a 
        # waypoint can be chosen with the bresenham path not
        # including collisions. 
        i, j = 0, 1
        pruned_path.append(path[i])
        found_max_path = False
        while j < len(path):
            p1 = path[i]
            p2 = path[j]
            # check if path includes collisions
            cells = list(bresenham(int(p1[0]),
                                   int(p1[1]),
                                   int(p2[0]),
                                   int(p2[1])))      
            # Check for obstacles
            if np.any([grid[x][y] for x,y in cells]):
                pruned_path.append(path[j-1])
                i = j
                j += 1
            else:
                j += 1
        pruned_path.append(path[j-1])
    else:
        pruned_path = [p for p in path]
        i = 0
        while i < len(pruned_path) - 2:
            p1 = point(pruned_path[i])
            p2 = point(pruned_path[i+1])
            p3 = point(pruned_path[i+2])
        
            if collinearity_check(p1, p2, p3,epsilon=12):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
    
    return pruned_path

