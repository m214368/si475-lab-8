#sudo pip install pydot==1.2.3
#sudo pip install decorator==4.4.2
#sudo pip install networkx=2.2.0

#sudo pip install setuptools==20.7.0
#pip 8.1.1

from math import pow, sqrt
import argparse
import networkx as nx
import matplotlib.pyplot as plt
from drive import Driver
from copy import deepcopy


def subtract(s1,s2):
    print(s1,s2)
    s1 = xy(s1)
    s2 = xy(s2)
    return sqrt(pow(s1[0]-s2[0],2)+pow(s1[1]-s2[1],2))

def xy(s1):
    s1 = s1[[x.isdigit() for x in s1].index(True):]
    s1 = s1.split(',')
    x = float(s1[0])
    y = float(s1[1][[:x.isdigit() for x in s1].index(False)])
    return x,y


class tb12:
    def __init__(self,startmap,endmap):
        filename = "map.dot"
        G = nx.drawing.nx_pydot.read_dot(filename)
        g = deepcopy(G) # only change G
        for key,value in startmap:
            G.add_node(key,label=value)
            best = 100
            bestnode = 0
            for n in list(g.nodes(data=True)):
                if (subtract(n[1]['label'],str(value)) < best):
                    best = subtract(n[1]['label'],str(value))
                    bestnode = n

        nx.set_edge_attributes(G, values = 1, name = 'weight')
        g = nx.Graph()
        self.driver = Driver()
        start = self.driver.start()
        s1 = "\""+str(start)+"\""
        bestend = 100
        bestendnode = 0
        e1 = "\""+str(args.e)+"\""
        for n in list(G.nodes(data=True)):
            g.add_node(n[0],label = n[1]['label'])
            if (subtract(n[1]['label'],s1) < beststart):
                beststart = subtract(n[1]['label'],s1)
                beststartnode = n
            if (subtract(n[1]['label'],e1) < bestend):
                bestend = subtract(n[1]['label'],e1)
                bestendnode = n
        for e in list(G.edges()):
            g.add_edge(e[0],e[1],weight=subtract(G.nodes[e[0]]['label'],G.nodes[e[1]]['label']))
        g.add_node("start",label=s1)
        g.add_edge("start",beststartnode[0],weight=beststart)
        g.add_node("end",label=e1)
        g.add_edge("end",bestendnode[0],weight=bestend)
        G = g
        pos=nx.spring_layout(G) # pos = nx.nx_agraph.graphviz_layout(G)
        nx.draw_networkx(G,pos)
        labels = nx.get_edge_attributes(G,'weight')
        nx.draw_networkx_edge_labels(G,pos,edge_labels=labels)
        #nx.draw(G)
        #plt.show()
        self.graph = G

    def path(self,start,end):
        length, path = nx.single_source_dijkstra(self.graph,start,target=end,weight='weight')
        return(length)

    def drive(self):
        length, path = nx.single_source_dijkstra(self.graph,"start",target="end",weight='weight')
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

print(xy("(\"1234,421321\")"))
