import numpy as np
import math
import json
from copy import deepcopy
from tb12 import tb12
from priqueue import PriQue

def dist(n1,n2):
    return math.sqrt((n1.x-n2.x)**2 +(n1.y-n2.y)**2)

def closest(n, N):
    X = []
    c = dist(n,N[0])
    cl = N[0].label
    for t in N:
        new_dist = dist(n,t)
        if new_dist < c:
            c = new_dist
            cl = t.label
    return cl

class Node:
    #goals:dictionary mapping robot,green,purple,blue,red to string locations
    goals = dict()
    def __init__(self,start=None,end=None):
        #parent:pointer to parent
        self.parent = None
        #action:action taken to get to that state
        self.action = None
        #location:dictionary mapping robot,green,purple,blue,red to string locations
        self.location = start
        #carry:list mapping 0,1 to balloons on the robot
        self.carry = list()
        #string representation of the state
        self.state = self.state()
        print(self.state)
        #goals:dictionary mapping of end states
        if (end is not None):
            goals = end 
        #value of the weight
        self.weight = 0
        self.map = tb12(start,self.goals)


    def drive(self,place): #place is a color or color_end
        node = Node(deepcopy(self.location))
        node.action = "drive " + place
        node.carry = self.carry
        node.parent = self
        node.location['ROBOT'] = node.location[place]
        print(node.location['ROBOT'])
        for i in node.carry:
            node.location[i] = place
        quarterback = tb12(self.location,self.goals)
        node.weight = quarterback.path('ROBOT',place)
        node.weight += self.weight
        return node

    def pickup(self,color):
        node = Node(deepcopy(self.location))
        node.action = "pickup " + color
        node.carry = self.carry
        node.parent = self
        node.carry.insert(color)
        node.carry.sort()
        node.weight = 3
        node.state = node.state()
        return node

    def putdown(self,color):
        node = Node(deepcopy(self.location))
        node.action = "putdown " + color
        node.carry = self.carry
        node.parent = self
        node.carry.remove(color)
        node.weight = 3
        node.state = node.state()
        return node

    def generate_children(self):
        childs = list()   
        #pickup all, putdown all, drive to balloons, drive to goals
        for i in self.location:
            temp = self.drive(i)
            childs.append(temp)
        for i in self.goals:
            temp = self.drive(i)
            childs.append(temp)
        for key,value in self.location.items():
            if value == self.location['ROBOT'] and key != 'ROBOT':
                temp = self.pickup(key)
                childs.append(temp)
        for i in self.carry:
            temp = self.putdown(i)
            childs.append(temp)
        print(childs)
        return childs

    def state(self):
        self.state = ""
        for key,value in sorted(self.location.items()):
            self.state += str(key)+":"+str(value)
        for key,value in sorted(self.carry):
            self.state += str(value)
        return self.state

    def heuristic(self):
        h = 0
        for key,value in self.goals.items():
            h+=tb12.subtract(value,self.location[key])
        h = h/2
        return h


class Graph:
    def __init__(self):
        #startFile = raw_input("Enter Starting State JSON File: ")
        startFile = "start.json"
        #endFile = raw_input("Enter Ending State JSON File: ")
        endFile = "simple.json"
        with open(startFile) as json_file:
            startLoc = json.load(json_file)
        with open(endFile) as json_file:
            endLoc = json.load(json_file)
        for key,value in startLoc.items():
            startLoc[key]=str(value)
        for key,value in endLoc.items():
            endLoc[key]=str(value)
        startN = Node(startLoc, endLoc)
        goal = Node(endLoc,endLoc)
        seen = dict()
        cur = startN
        pq = PriQue()
        #A* search stuff
        while cur.state != goal.state:
            for next in cur.generate_children():
                if next.state not in seen or True:
                    pq.insert(next.weight + next.heuristic(), next)
                    seen[next.state] = next.weight + next.heuristic()
                #else:
                #    if seen[next.state()] > next.weight + next.heuristic():
                #        seen[next.state()] = next.weight + next.heuristic()
                #        pq.changePriority(next.weight + next.heuristic(), next)
                #pq.printMe()
            cur =  pq.getMin()
        path = []
        #get path
        path.insert(0, cur.action)
        while cur.parent is not None:
            path.insert(0, cur.parent.action)
            cur = cur.parent
        print path
        return path

g = Graph()