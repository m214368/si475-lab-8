import numpy as np
import math
import json
from copy import deepcopy
from tb12 import tb12
from priqueue import PriQue
import re

def dist(n1,n2):
    return math.sqrt((n1[0]-n2[0])**2 +(n1[1]-n2[1])**2)

def xy(s1):
    s1 = re.findall( r'\d+\.*\d*', s1 )
    x = float(s1[0])
    y = float(s1[1])
    return x,y

class Node:
    def __init__(self, start, goals, robot):
        self.locations = start
        self.goals = goals
        self.parent = None
        self.action = None
        self.weight = 0
        self.robot = robot

    def drive(self, place, locs):
        node = Node(deepcopy(self.locations), deepcopy(self.goals), deepcopy(self.robot))
        node.action = "drive " + str(locs[place])
        node.parent = self
        node.robot.loc = locs[place]
        #update balloon locations
        for i in self.robot.carrying:
            node.locations[i] = locs[place]
        node.weight = dist(self.robot.loc, locs[place])
        return node

    def pickup(self, color):
        node = Node(deepcopy(self.locations), deepcopy(self.goals), deepcopy(self.robot))
        node.action = "pickup " + color
        node.robot.pickup(color)
        node.weight = 0
        node.parent = self
        return node

    def putdown(self, color):
        node = Node(deepcopy(self.locations), deepcopy(self.goals), deepcopy(self.robot))
        node.action = "putdown " + color
        node.robot.putdown(color)
        node.locations[color] = node.robot.loc
        node.weight = 0
        node.parent = self
        return node

    def heuristic(self):
        h = 0
        for key,value in self.goals.items():
            h+=dist(value,self.locations[key])
        h = h/2
        return h
    
    def generate_children(self):
        childs = list()
        if len(self.robot.carrying) < 2:
            for i in self.locations:
                temp = self.drive(i, self.locations)
                childs.append(temp)
            for i in self.locations:
                if self.locations[i] == self.robot.loc and i not in self.robot.carrying:
                    temp = self.pickup(i)
                    childs.append(temp)
        for i in self.robot.carrying:
            if self.robot.loc is not self.goals[i]:
                temp = self.drive(i, self.goals)
                childs.append(temp)
        for i in self.robot.carrying:
            if self.goals[i] == self.robot.loc:
                temp = self.putdown(i)
                childs.append(temp)
        return childs

    def State(self):
        self.state = ""
        for key,value in sorted(self.locations.items()):
            self.state += str(key)+":"+str(value)
        for value in sorted(self.robot.carrying):
            self.state += "carrying:"+str(value)
        return self.state
    def RState(self):
        self.state = ""
        for key,value in sorted(self.locations.items()):
            self.state += str(key)+":"+str(value)
        for value in sorted(self.robot.carrying):
            self.state += "carrying:"+str(value)
        self.state += " ROBOT: " + str(self.robot.loc)
        return self.state

    def goalState(self):
        self.state = ""
        for key,value in sorted(self.goals.items()):
            self.state += str(key)+":"+str(value)
        for value in sorted(self.robot.carrying):
            self.state += "carrying:"+str(value)
        return self.state

class Rob:
    def __init__(self, start):
        self.loc = start
        self.carrying = []
    def pickup(self, color):
        self.carrying.append(color)
    def putdown(self, color):
        #print("Putting down "+str(color))
        self.carrying.remove(color) 
        #print(self.carrying)


class Searcher:
    def search(self):
        #start file
        startFile = "start.json"
        endFile = "harder.json"
        with open(startFile) as json_file:
            startLoc = json.load(json_file)
        with open(endFile) as json_file:
            endLoc = json.load(json_file)
	bot = tb12()
	#get starting location of robot
        startRob = bot.driver.start()
	#start = (0,0)
	#only need to get x,y
        robot = Rob(startRob)
        startN = Node(startLoc, endLoc, robot)
        seen = dict()
        cur = deepcopy(startN)
        pq = PriQue()
        while cur.State() != startN.goalState():
            for next in cur.generate_children():
            #print next.action
                if next.State() not in seen:
                    pq.insert(next.weight+next.heuristic(), next)
                    seen[next.State()] = next.weight
            cur = pq.getMin()

        path = []
        #get path
        path.insert(0, cur.action)
        cur = cur.parent
        while cur.parent is not None:
            path.insert(0, cur.action)
            cur = cur.parent
        print path
        return path
    
s = Searcher()
path = s.search()
bot = tb12()
for i in path:
	next = i[1]
	color = i.split()[-1]
	if next == "r":
		x,y = xy(i)
		bot.drive(x,y)
	if next == "i":
		bot.pickup(color)
	if next == "u":
		bot.putdown(color)

