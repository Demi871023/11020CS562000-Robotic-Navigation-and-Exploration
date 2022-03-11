from ctypes import util
from dis import dis
from hashlib import new
from random import paretovariate
from typing_extensions import Self
import cv2
import numpy as np
import sys
sys.path.append("..")
import PathPlanning.utils as utils
from PathPlanning.planner import Planner

class PlannerRRTStar(Planner):
    def __init__(self, m, extend_len=20):
        super().__init__(m)
        self.extend_len = extend_len 

    def _random_node(self, goal, shape):
        r = np.random.choice(2,1,p=[0.5,0.5])
        if r==1:
            return (float(goal[0]), float(goal[1]))
        else:
            rx = float(np.random.randint(int(shape[1])))
            ry = float(np.random.randint(int(shape[0])))
            return (rx, ry)

    def _nearest_node(self, samp_node):
        min_dist = 99999
        min_node = None
        for n in self.ntree:
            dist = utils.distance(n, samp_node)
            if dist < min_dist:
                min_dist = dist
                min_node = n
        return min_node

    def _check_collision(self, n1, n2):
        n1_ = utils.pos_int(n1)
        n2_ = utils.pos_int(n2)
        line = utils.Bresenham(n1_[0], n2_[0], n1_[1], n2_[1])
        for pts in line:
            if self.map[int(pts[1]),int(pts[0])]<0.5:
                return True
        return False

    def _steer(self, from_node, to_node, extend_len):
        vect = np.array(to_node) - np.array(from_node)
        v_len = np.hypot(vect[0], vect[1])
        v_theta = np.arctan2(vect[1], vect[0])
        if extend_len > v_len:
            extend_len = v_len
        new_node = (from_node[0]+extend_len*np.cos(v_theta), from_node[1]+extend_len*np.sin(v_theta))
        if new_node[1]<0 or new_node[1]>=self.map.shape[0] or new_node[0]<0 or new_node[0]>=self.map.shape[1] or self._check_collision(from_node, new_node):
            return False, None
        else:        
            return new_node, utils.distance(new_node, from_node)

    def _get_near_node_set(self, near_node, radius):
        near_nodes_set = []
        for node in self.ntree:
            if self._check_collision(near_node, node):
                continue
            dist = utils.distance(near_node, node)
            if dist < radius:
                near_nodes_set.append(node)
        return near_nodes_set

    def _get_best_parent(self, now_parent, near_node_set, new_node):
        parent = now_parent
        for node in near_node_set:
            if self.cost[now_parent] + utils.distance(now_parent, new_node) > self.cost[node] + utils.distance(node, new_node):
                parent = node
        return parent

    def _rewrite(self, best_parent, near_node_set, new_node):
        for node in near_node_set:
            if node == best_parent:
                continue
            else:
                if self.cost[node] > self.cost[new_node] + utils.distance(node, new_node):
                    self.cost[node] = self.cost[new_node] + utils.distance(node, new_node)
                    self.ntree[node] = new_node

    

    

    def planning(self, start, goal, extend_len=None, img=None):
        if extend_len is None:
            extend_len = self.extend_len
        self.ntree = {}
        self.ntree[start] = None
        self.cost = {}
        self.cost[start] = 0
        goal_node = None
        for it in range(20000):
            #print("\r", it, len(self.ntree), end="")
            samp_node = self._random_node(goal, self.map.shape)             # (1) Sample points on 2d space
            near_node = self._nearest_node(samp_node)                       # (2) Find the nearest point in graph
            new_node, cost = self._steer(near_node, samp_node, extend_len)  # (3) extend the branch from the near_node to samp_node extend_len to get new_node
            if new_node is not False:
                self.ntree[new_node] = near_node
                self.cost[new_node] = cost + self.cost[near_node]
            else:
                continue
            if utils.distance(near_node, goal) < extend_len:
                goal_node = near_node
                break
                
            # TODO: Re-Parent & Re-Wire

            # Find the near nodes set of the new node in a distance range
            near_node_set = self._get_near_node_set(new_node , 500)
            best_parent = self._get_best_parent(near_node, near_node_set, new_node)

            self.ntree[new_node] = best_parent
            self.cost[new_node] = utils.distance(best_parent, new_node) + self.cost[best_parent]

            self._rewrite(best_parent, near_node_set, new_node)


            # Draw
            if img is not None:
                for n in self.ntree:
                    if self.ntree[n] is None:
                        continue
                    node = self.ntree[n]
                    cv2.line(img, (int(n[0]), int(n[1])), (int(node[0]), int(node[1])), (0,1,0), 1)
                # Near Node
                img_ = img.copy()
                cv2.circle(img_,utils.pos_int(new_node),5,(0,0.5,1),3)
                # Draw Image
                img_ = cv2.flip(img_,0)
                cv2.imshow("Path Planning",img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break
        
        # Extract Path
        path = []
        n = goal_node
        while(True):
            if n is None:
                break
            path.insert(0,n)
            node = self.ntree[n]
            n = self.ntree[n] 
        path.append(goal)
        return path