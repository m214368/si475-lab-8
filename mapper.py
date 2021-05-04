from math import pow, sqrt
import argparse
import networkx as nx
import matplotlib.pyplot as plt
#from drive import Driver
from copy import deepcopy
import re

class Mapper:
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
        for n in list(G.nodes(data=True)):
            g.add_node(n[0],label = n[1]['label'])
        for e in list(G.edges()):
            g.add_edge(e[0],e[1],weight=subtract(G.nodes[e[0]]['label'],G.nodes[e[1]]['label']))
        G = g
        self.graph = G


    def heuristic(startmap,endmap):
        return 5

    def weight(startloc,endloc):
        return 10
