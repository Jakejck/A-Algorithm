import copy

def shortest_path(M,start,goal):
    pathplanner_object = PathPlanner(M,start,goal)
    return pathplanner_object.path

# Creat Class 
class PathPlanner():
    """Construct a PathPlanner Object"""
    def __init__(self, M, start=None, goal=None):
        self.map = M
        self.start= start
        self.goal = goal
        self.closedList = self.create_closedList() if goal != None and start != None else None
        self.openQueue = self.create_openQueue() if goal != None and start != None else None
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None
  
   
    def _reset(self):
        """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
        self.closedList = None
        self.openQueue = None
        self.cameFrom = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        """ """
        if self.map == None:
            raise(ValueError, "Must create map before running search. Try running PathPlanner.set_map(M)")
        if self.goal == None:
            raise(ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(goal_node)")
        if self.start == None:
            raise(ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.closedList = self.closedList if self.closedList != None else self.create_closedList()
        self.openQueue = self.openQueue if self.openQueue != None else  self.create_openQueue()
        self.cameFrom = self.cameFrom if self.cameFrom != None else  self.create_cameFrom()
        
        while not self.is_open_empty():
            current = self.get_current_node() 

            
            if current == self.goal:
                for neighbor in self.get_neighbors(current):
                    if neighbor in self.openQueue.keys():
                        current_gscore_from_neighbor = self.get_tentative_gScore(neighbor,current)
                        if current_gscore_from_neighbor < self.openQueue[current][1]:
                            self.openQueue[current][1] = current_gscore_from_neighbor
                            self.openQueue[current][0] = self.calculate_fscore(current)
                            self.openQueue[current][2] = copy.deepcopy(self.openQueue[neighbor][2])
                            self.openQueue[current][2].append(current)
                self.cameFrom = self.openQueue[current][2]
                return self.cameFrom
                
            else:
                for neighbor in self.get_neighbors(current):
                    if neighbor in self.closedList:
                        continue    # Ignore the neighbor which is already evaluated.
                    else:   # Discover a new node
                        neighbor_gScore_from_current = self.get_tentative_gScore(current,neighbor)
                        if neighbor in self.openQueue.keys():
                            if neighbor_gScore_from_current < self.openQueue[neighbor][1]:
                                self.openQueue[neighbor][1] = neighbor_gScore_from_current
                                self.openQueue[neighbor][0] = self.calculate_fscore(neighbor)
                                self.openQueue[neighbor][2] = copy.deepcopy(self.openQueue[current][2])
                                self.openQueue[neighbor][2].append(neighbor)
                        else:
                            self.openQueue[neighbor] = [0,neighbor_gScore_from_current,[]]
                            self.openQueue[neighbor][0] = self.calculate_fscore(neighbor)
                            self.openQueue[neighbor][2] = copy.deepcopy(self.openQueue[current][2])
                            self.openQueue[neighbor][2].append(neighbor)
                          
                            
                            for sub_neighbor in self.get_neighbors(neighbor):
                                if sub_neighbor in self.openQueue.keys():
                                    neighbor_gscore_from_sub_neighbor = self.get_tentative_gScore(sub_neighbor,neighbor)
                                    if neighbor_gscore_from_sub_neighbor < self.openQueue[neighbor][1]:
                                        self.openQueue[neighbor][1] = neighbor_gscore_from_sub_neighbor
                                        self.openQueue[neighbor][0] = self.calculate_fscore(neighbor)
                                        self.openQueue[neighbor][2] = copy.deepcopy(self.openQueue[sub_neighbor][2])
                                        self.openQueue[neighbor][2].append(neighbor)
                self.closedList.append(current)
                del self.openQueue[current]
                
                
# Data Structures
def create_closedList(self):
    """ Creates and returns a data structure suitable to hold the set of nodes already evaluated"""
    # EXAMPLE: return a data structure suitable to hold the set of nodes already evaluated
    return []

def create_openQueue(self):
    """ Creates and returns a data structure suitable to hold the set of currently discovered nodes 
    that are not evaluated yet. Initially, only the start node is known."""
    if self.start != None:
        # TODO: return a data structure suitable to hold the set of currently discovered nodes 
        # that are not evaluated yet. Make sure to include the start node.

        openQueue=dict()
        openQueue[self.start] = [self.heuristic_cost_estimate(self.start),self.distance(self.start,self.start),[self.start]]
        return openQueue
    
    raise(ValueError, "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")
    
def create_cameFrom(self):
    """Creates and returns a data structure that shows which node can most efficiently be reached from another,
    for each node."""
    # TODO: return a data structure that shows which node can most efficiently be reached from another,
    # for each node.
    return []

#Get node information
def is_open_empty(self):
    """returns True if the open set is empty. False otherwise. """
    # TODO: Return True if the open set is empty. False otherwise.
    if len(self.openQueue) == 0 :
        return True
    else:
        return False

def get_current_node(self):
    """ Returns the node in the open set with the lowest value of f(node)."""
    # TODO: Return the node in the open set with the lowest value of f(node).
    return min(self.openQueue,key = self.openQueue.get)

def get_neighbors(self, node):
    """Returns the neighbors of a node"""
    # TODO: Return the neighbors of a node
    neighbors = self.map.roads[node]
    return neighbors

#Set certain variables

def set_map(self, M):
    """Method used to set map attribute """
    self._reset(self)
    self.start = None
    self.goal = None
    # TODO: Set map to new value.
    self.map = M
    
def set_start(self, start):
    """Method used to set start attribute """
    self._reset(self)
    # TODO: Set start value. Remember to remove goal, closedSet, openSet, cameFrom, gScore, fScore, 
    # and path attributes' values.
    self.start = start
    
def set_goal(self, goal):
    """Method used to set goal attribute """
    self._reset(self)
    # TODO: Set goal value.
    self.goal = goal
    
#Scores and Costs
def get_gScore(self, node):
    """Returns the g Score of a node"""
    # TODO: Return the g Score of a node
    return self.openQueue[node][1]

def distance(self, node_1, node_2):
    """ Computes the Euclidean L2 Distance"""
    # TODO: Compute and return the Euclidean L2 Distance
    distance = ((self.map.intersections[node_1][0] - self.map.intersections[node_2][0])**2 + 
                (self.map.intersections[node_1][1] - self.map.intersections[node_2][1])**2)**0.5
    return distance

def heuristic_cost_estimate(self, node):
    """ Returns the heuristic cost estimate of a node """
    # TODO: Return the heuristic cost estimate of a node
    return self.distance(node,self.goal)

def get_tentative_gScore(self, current, neighbor):
    """Returns the tentative g Score of a node"""
    # TODO: Return the g Score of the current node 
    # plus distance from the current node to it's neighbors
    return self.get_gScore(current) + self.distance(current,neighbor)

def calculate_fscore(self, node):
    """Calculate the f score of a node. """
    # TODO: Calculate and returns the f score of a node. 
    # REMEMBER F = G + H
    return self.get_gScore(node) + self.heuristic_cost_estimate(node)

# Associating your functions with the PathPlanner class

PathPlanner.create_closedList = create_closedList
PathPlanner.create_openQueue = create_openQueue
PathPlanner.create_cameFrom = create_cameFrom
PathPlanner.is_open_empty = is_open_empty
PathPlanner.get_current_node = get_current_node
PathPlanner.get_neighbors = get_neighbors
PathPlanner.set_map = set_map
PathPlanner.set_start = set_start
PathPlanner.set_goal = set_goal
PathPlanner.get_gScore = get_gScore
PathPlanner.distance = distance
PathPlanner.heuristic_cost_estimate = heuristic_cost_estimate
PathPlanner.get_tentative_gScore = get_tentative_gScore
PathPlanner.calculate_fscore = calculate_fscore



