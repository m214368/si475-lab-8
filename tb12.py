#sudo pip install pydot==1.2.3
#sudo pip install decorator==4.4.2
#sudo pip install networkx=2.2.0

#sudo pip install setuptools==20.7.0
#pip 8.1.1

from math import pow, sqrt
import argparse
import networkx as nx
import matplotlib.pyplot as plt
#from drive import Driver
from copy import deepcopy
import re


def subtract(s1,s2):
    #print(s1,s2)
    s1 = xy(s1)
    s2 = xy(s2)
    return sqrt(pow(s1[0]-s2[0],2)+pow(s1[1]-s2[1],2))

def xy(s1):
    s1 = re.findall( r'\d+\.*\d*', s1 )
    x = float(s1[0])
    y = float(s1[1])
    return x,y


class tb12:
    driver = None#Driver()
    def __init__(self,startmap,endmap):
        filename = "map.dot"
        G = nx.drawing.nx_pydot.read_dot(filename)
        g = G.__class__()
        g.add_nodes_from(G.nodes(data=True))
        g.add_edges_from(G.edges)
        for key,value in startmap.items():
            G.add_node(str(key),label=value)
            best = 100
            bestnode = 0
            for n in list(g.nodes(data=True)):
                if (subtract(n[1]['label'],str(value)) < best):
                    best = subtract(n[1]['label'],str(value))
                    bestnode = n
            G.add_edge(key,bestnode[0],weight=best)
        for key,value in endmap.items():
            G.add_node(str(key)+"_END",label=value)
            best = 100
            bestnode = 0
            for n in list(g.nodes(data=True)):
                if (subtract(n[1]['label'],str(value)) < best):
                    best = subtract(n[1]['label'],str(value))
                    bestnode = n
            G.add_edge(key,bestnode[0],weight=best)
        nx.set_edge_attributes(G, values = 1, name = 'weight')
        g = nx.Graph()
        startmap["ROBOT"] = "(14,13)"
        try:
            start = startmap["ROBOT"]
        except:
            start = driver.start()
        s1 = "\""+str(start)+"\""
        best = 100
        bestnode = 0
        for n in list(G.nodes(data=True)):
            g.add_node(n[0],label = n[1]['label'])
            if (subtract(n[1]['label'],s1) < best):
                best = subtract(n[1]['label'],s1)
                bestnode = n
        for e in list(G.edges()):
            g.add_edge(e[0],e[1],weight=subtract(G.nodes[e[0]]['label'],G.nodes[e[1]]['label']))
        g.add_node("ROBOT",label=s1)
        g.add_edge("ROBOT",bestnode[0],weight=best)
        G = g
        self.graph = G

    def path(self,start,end):
        length, path = nx.single_source_dijkstra(self.graph,start,target=end,weight='weight')
        return(length) 

    def drive(self,end):
        length, path = nx.single_source_dijkstra(self.graph,"ROBOT",target=end,weight='weight')
        for i in path:
            x,y = xy(self.graph.nodes[i]['label'])
            self.driver.goto(x,y)

    def start(self):
        return self.driver.start()

    def pickup(self,color):
        print("pick up " + color + " balloon.")
        self.driver.pickup(color)

    def putdown(self,color):
        print("put down " + color + " balloon.")

