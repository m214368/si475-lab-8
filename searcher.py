def xy(s1):
    s1 = re.findall( r'\d+\.*\d*', s1 )
    x = float(s1[0])
    y = float(s1[1])
    return x,y

class Node:
    def __init__(self,startmap,endmap,robot,carrying=list(),pparent=None,pweight=0,paction=None):
        self.start = startmap
        self.end = endmap
        self.robot = robot
        self.parent = pparent
        self.carrying = carrying
        self.weight = pweight
        self.action = paction
        self.map = Mapper(startmap,endmap)

    def drive(self,place): #place is a color or color_end
        action = "drive "+ locs[place]
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


    def UpdateWeight(self,mapper,startloc,endloc):
        self.weight += mapper.weight(startloc,endloc)

    def Heuristic(self,mapper):
        return mapper.heuristic(self.start,self.end)

    def State(self):
        self.state = ""
        for key,value in sorted(self.start.items()):
            self.state += str(key)+":"+str(value)
        for value in sorted(self.carry):
            self.state += "carrying:"+str(value)
        self.state+="ROBOT:"+str(robot)
        return self.state

    def Check(self):
        self.state = ""
        for key,value in sorted(self.start.items()):
            self.state += str(key)+":"+str(value)
        for value in sorted(self.carry):
            self.state += "carrying:"+str(value)
        return self.state

class Searcher:
    def search():
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
        #drive robot to first location
        #driver = Driver()
        mapper = Mapper(startL,endL)
        #start = str(driver.start())
        start = "15,4"
        best = 100
        bestnode = 0
        for n in list(mapper.graph.nodes(data=True)):
            if (subtract(n[1]['label'],start) < best):
                best = subtract(n[1]['label'],start)
                bestnode = n
        x,y = xy(bestnode['label'])
        #driver.goto(x,y)
        robot = bestnode['label']

        startN = Node(mapper,startL, endL,robot)
        goal = Node(Mapper(startL,endL),endL,endL,robot)
        seen = dict()
        cur = startN
        pq = PriQue()
        #A* search stuff
        while cur.Check() != goal.Check():
            for next in Searcher.generate_children(cur):
                if next.State() not in seen:
                    pq.insert(next.weight + next.heuristic(), next)
                    seen[next.State()] = next.weight + next.heuristic()
            cur =  pq.getMin()
        path = []
        #get path
        path.insert(0, cur.action)
        while cur.parent is not None:
            path.insert(0, cur.parent.action)
            cur = cur.parent
        print path
        return path

    def generate_children(node):
        childs = list()
        if len(node.carry) < 2:
            for i in node.start:
                temp = Node(node.startmap,node.endmap,robot,carrying=list(),pparent=None,pweight=0,paction=None):
                childs.append(temp)
            for key,value in self.start.items():
                if value==self.robot and not in node.carry:
                    temp = self.pickup(key)
                    childs.append(temp)
        for i in node.carry:
            temp = self.drive(robot, i+"_END")
            childs.append(temp)
            temp = self.putdown(i)
            childs.append(temp)
        return childs
