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
        self.state = self.State()
        #print(self.state)
        #goals:dictionary mapping of end states
        if (end is not None):
            goals = end
        #value of the weight
        self.weight = 0
        self.map = tb12(start,self.goals)


    def drive(self,place, locs): #place is a color or color_end
        node = Node(deepcopy(self.location), deepcopy(self.goals))
        node.action = "drive " + locs[place]
        node.carry = self.carry
        node.parent = self
        node.location['ROBOT'] = locs[place]
        #print(node.location['ROBOT'])
        #print(node.carry)
        for i in node.carry:
            node.location[i] = locs[place]
        quarterback = tb12(self.location,self.goals)
        node.weight = quarterback.path('ROBOT',place)
        node.weight += self.weight
        #print "drive"
        #print node.location
        return node

    def pickup(self,color):
        node = Node(deepcopy(self.location))
        node.action = "pickup " + color
        node.carry = self.carry
        node.parent = self
        node.carry.insert(0,color)
        node.carry.sort()
        node.weight = 3
        node.state = node.State()
        #print("pickup")
        #print node.State()
        return node

    def putdown(self,color):
        node = Node(deepcopy(self.location))
        node.action = "putdown " + color
        node.carry = self.carry
        node.parent = self
        node.carry.remove(color)
        node.weight = 3
        node.state = node.State()
        #print "putdown"
        return node

    def generate_children(self, goals):
        childs = list()
        #pickup all, putdown all, drive to balloons, drive to goals
        #print(self.location)
        for i in self.location:
            temp = self.drive(i, self.location)
        #print temp.location['ROBOT']
            childs.append(temp)
        for i in goals:
            temp = self.drive(i, goals)
            childs.append(temp)
        for i in self.carry:
            temp = self.putdown(i)
            childs.append(temp)
    for key,value in self.location.items():
            #print("value: "+str(value))
            #print("Robot:"+str(self.location["ROBOT"]))
            if value==self.location["ROBOT"] and key != 'ROBOT':
                #print key
                temp = self.pickup(key)
                childs.append(temp)
        #print(childs)
        return childs

    def State(self):
        self.state = ""
        for key,value in sorted(self.location.items()):
            self.state += str(key)+":"+str(value)
        for value in sorted(self.carry):
            self.state += "carrying:"+str(value)
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
        startFile = "start1.json"
        #endFile = raw_input("Enter Ending State JSON File: ")
        endFile = "simple1.json"
        with open(startFile) as json_file:
            startLoc = json.load(json_file)
        with open(endFile) as json_file:
            endLoc = json.load(json_file)
        startL = dict()
        endL = dict()
        for key,value in startLoc.items():
            startL[str(key.encode('ascii','ignore'))]=str(value)
        for key,value in endLoc.items():
            endL[str(key.encode('ascii','ignore'))]=str(value)
        startL['ROBOT'] = "[14,13]"
        endL['ROBOT'] = "[14,13]"
        startN = Node(startL, endL)
        goal = Node(endL,endL)
        #print goal.state
        seen = dict()
        cur = startN
        pq = PriQue()
    print "/////////////////////////////"
        #A* search stuff
        while cur.State() != goal.state:
        print cur.State()
            print "////////////////////////////"
            for next in cur.generate_children(goal.location):
                #print next.State()
                #print next.weight
                if next.State() not in seen:
                    pq.insert(next.weight + next.heuristic(), next)
                    seen[next.State()] = next.weight + next.heuristic()
                #else:
                #    if seen[next.State()] > next.weight + next.heuristic():
                #       seen[next.State()] = next.weight + next.heuristic()
                #       pq.insert(next.weight + next.heuristic(), next)
                #pq.printMe()
            cur =  pq.getMin()
        print "////////////"
        path = []
        #get path
        path.insert(0, cur.action)
        while cur.parent is not None:
            path.insert(0, cur.parent.action)
            cur = cur.parent
        print path
        return path

g = Graph()
