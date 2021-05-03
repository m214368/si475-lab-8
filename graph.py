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
        #goals:dictionary mapping of end states
        goals = end 
        #value of the weight
        self.weight = 0


    def drive(self,place):
        node = Node()
        node.action = "drive " + place
        node.location = deepcopy(self.location)
        node.carry = self.carry
        node.parent = self
        node.location[robot] = place
        for i in node.carry:
            node.location[i] = place
        node.weight = tb12.path(self.location[robot],place)
        return node

    def pickup(self,color):
        node = Node()
        node.action = "pickup " + color
        node.location = deepcopy(self.location)
        node.carry = self.carry
        node.parent = self
        node.carry.insert(color)
        node.carry.sort()
        node.weight = 3
        node.state = node.state()
        return node

    def putdown(self,color):
        node = Node()
        node.action = "putdown " + color
        node.location = deepcopy(self.location)
        node.carry = self.carry
        node.parent = self
        node.carry.remove(color)
        node.weight = 3
        node.state = node.state()
        return node

    def generate_children(self):
        childs = []
        #marks the first node as seen
        if self.parent == None:
            seen[self.state] = True
        #pickup all, putdown all, drive to balloons, drive to goals
        for i in self.location:
	    #create node
            temp = self.drive(i)
            #add node to our list
	    childs.append(temp)
            #this logic goes into the graph side
            '''
                if not temp.seen(): #should really just reduce the priqueue value of the seen
                    priority queue add temp
                    seen[temp.state] = True
            '''
        for i in goals:
            temp = self.drive(i)
            childs.append(temp)
            #this logic goes into the graph side
            '''
                if not temp.seen():
                    priority queue add temp #should really just reduce the priqueue value of the seen
                    seen[temp.state] = True
            '''
        for key,value in self.location.items():
            if value == location['robot'] and key != 'robot':
                temp = self.pickup(key)
                childs.append(temp)
                #this logic goes into the graph side
                '''
                if not temp.seen():
                    priority queue add temp #should really just reduce the priqueue value of the seen
                    seen[temp.state] = True
                '''
        for i in self.carry:
            temp = self.putdown(i)
            childs.append(temp)
            #this logic goes into the graph side
            '''
            if not temp.seen():
                priority queue add temp #should really just reduce the priqueue value of the seen
                seen[temp.state] = True
            '''
        return childs

    def state(self):
        self.state = ""
        for key,value in sorted(self.location.items()):
            self.state += str(value)
        for key,value in sorted(self.carry):
            self.state += str(value)
        return self.state

    def seen(self):
        return self.state in seen.keys()

    def heuristic(self):
        h = 0
        for key,value in goals.items:
            h+=tb12.subtract(value,self.location[key])
        h = h/2
        return h


class Graph:
    def __init__(self):
        self.start = None
        self.goal = None

startFile = raw_input("Enter Starting State JSON File: ")
endFile = raw_input("Enter Ending State JSON File: ")

with open(startFile) as json_file:
  startLoc = json.load(json_file)

with open(endFile) as json_file:
  endLoc = json.load(json_file)

quarterback = tb12(startLoc,endLoc)
start['ROBOT'] = tb12.start()
startN = Node(startLoc, endLoc)
seen = dict()
startN.weight = 0
cur = startN
pq = PriQue()
#A* search stuff
while cur.location != startN.goal:
  for next in cur.generate_children():
    if not next.seen():
      next.weight = cur.weight + #cost?
      pq.append(next.weight + next.heuristic(), next)
      seen.append(next.state)
    else:
     if next.weight > cur.weight + #cost?:
       next.weight = cur.weight + #cost
       pq.changePriority(next.weight + next.heuristic(), next)
       
  cur =  pq.getMin()

path = []
#get path
path.insert(0, cur.action)
while cur.parent != None:
  path.insert(0, cur.parent.action)
  cur = cur.parent

print path
      





