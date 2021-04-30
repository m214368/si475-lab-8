import numpy as np
import math
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
    goals
    seen = dict()
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
        for i in node.carry
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
        #marks the first node as seen
        if self.parent == None:
            seen[self.state] = True
        #pickup all, putdown all, drive to balloons, drive to goals
        for i in self.location:
            temp = self.drive(i)
            #this logic goes into the graph side
            '''
                if not temp.seen(): #should really just reduce the priqueue value of the seen
                    priority queue add temp
                    seen[temp.state] = True
            '''
        for i in goals:
            temp = self.drive(i)
            #this logic goes into the graph side
            '''
                if not temp.seen():
                    priority queue add temp #should really just reduce the priqueue value of the seen
                    seen[temp.state] = True
            '''
        for key,value in self.location.items():
            if value == location['robot'] and key != 'robot':
                temp = self.pickup(value)
                #this logic goes into the graph side
                '''
                if not temp.seen():
                    priority queue add temp #should really just reduce the priqueue value of the seen
                    seen[temp.state] = True
                '''
        for i in self.carry:
            temp = self.putdown(i)
            #this logic goes into the graph side
            '''
            if not temp.seen():
                priority queue add temp #should really just reduce the priqueue value of the seen
                seen[temp.state] = True
            '''

    def state(self):
        self.state = ""
        for key,value in sorted(self.location.items()):
            self.state += str(value)
        for key,value in sorted(self.carry.items()):
            self.state += str(value)
        print(self.state)

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
        self.vert = []
        self.adj_mtrx = []
        self.label_to_i = {}
        self.start = None
        self.goal = None

    def build_from_dot(self,filename,startx,starty,goalx,goaly):
        f = open(filename)
        f.readline()
        edges = set()
        for line in f:
            line1 = line.split('[')
            line2 = line.split('--')
            if len(line1)==2:
                name = line1[0].replace(' ','')
                points = line1[1].split('"')[1]
                points = points.split(',')
                x = float(points[0].replace('(',''))
                y = float(points[1].replace(')',''))
                self.label_to_i[name] = len(self.vert)
                self.vert.append(Node(name,x,y))
            if len(line2)==2:
                edges.add((line2[0].strip(),
                    line2[1].replace(';','').strip()))

        start = Node('start',startx,starty)
        goal = Node('goal',goalx,goaly)
        edges.add(('start',closest(start,self.vert)))
        edges.add(('goal',closest(goal,self.vert)))
        self.label_to_i['start'] = len(self.vert)
        self.vert.append(start)
        self.label_to_i['goal'] = len(self.vert)
        self.vert.append(goal)

        self.adj_mtrx = np.zeros((len(self.vert),len(self.vert)))-1
        for e in edges:
            i = self.label_to_i[e[0]]
            j = self.label_to_i[e[1]]
            d = dist(self.vert[i],self.vert[j])
            self.adj_mtrx[i][j] = d
            self.adj_mtrx[j][i] = d

        #print(self.vert)
        #print(self.adj_mtrx)

        f.close()

    def get_neighbors(self, vertex):
        nbrs = []
        row = self.adj_mtrx[self.label_to_i[vertex.label]]
        for i in range(len(row)):
            if row[i] != -1:
                nbrs.append((self.vert[i],row[i]))
        return nbrs

    def get_start(self):
        return self.vert[-2]

    def get_goal(self):
        return self.vert[-1]
