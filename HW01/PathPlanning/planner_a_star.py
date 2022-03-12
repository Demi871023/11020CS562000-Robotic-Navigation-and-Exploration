import cv2
import sys
sys.path.append("..")
import PathPlanning.utils as utils
from PathPlanning.planner import Planner

class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.inter = inter
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None

    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        if inter is None:
            inter = self.inter
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        # Initialize 
        self.initialize()
        self.queue.append(start)
        self.parent[start] = None
        self.g[start] = 0
        self.h[start] = utils.distance(start, goal)
        while(1):
            # TODO: A Star Algorithm
            cost_min = 99999
            parent_node = start
            next_node = start

            cost_g = 0
            cost_h = 0

            for node in self.queue:
                # search each node neighbor
                neighborhood = [
                    (int(node[0]), int(node[1] + inter)),           # up
                    (int(node[0]), int(node[1] - inter)),           # down
                    (int(node[0] - inter), int(node[1])),           # left
                    (int(node[0] + inter), int(node[1])),           # right
                    (int(node[0] - inter), int(node[1] + inter)),   # left-up
                    (int(node[0] + inter), int(node[1] + inter)),   # right-up
                    (int(node[0] - inter), int(node[1] - inter)),   # left-down
                    (int(node[0] + inter), int(node[1] - inter)),   # right-down
                ]

                for neighbor in neighborhood:

                    # check if the wall
                    if self.map[int(neighbor[1]),int(neighbor[0])] < 0.5:
                        continue

                    # calculate each might node cost
                    if neighbor not in self.queue:
                        # self.g[neighbor] = self.g[node] + inter
                        # self.h[neighbor] = utils.distance(neighbor, goal)
                        cost_g = self.g[node] + inter
                        cost_h = utils.distance(neighbor, goal)
                        # self.parent[neighbor] = node
                    else:
                        continue

                    # find which might node is smallest
                    cost = cost_g + cost_h
                    if cost < cost_min:
                        cost_min = cost
                        next_node = neighbor
                        parent_node = node
                
            # add smallest node into queue
            self.g[next_node] = self.g[parent_node] + inter
            self.h[next_node] = utils.distance(parent_node, goal)
            self.parent[next_node] = parent_node
            self.queue.append(next_node)     

            # if smallest node is too near with goal then finish search
            if utils.distance(next_node, goal) < inter:
                self.goal_node = next_node
                break

        # draw all node in queue
        circle_radius = 2
        circle_color = (0, 0, 255)
        circle_thickness = 1
        for node in self.queue:
            img = cv2.circle(img, node, circle_radius, circle_color, circle_thickness)
        
        # Extract path
        path = []
        p = self.goal_node
        if p is None:
            return path
        while(True):
            path.insert(0,p)
            if self.parent[p] is None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path
