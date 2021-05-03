class PriQue(object):
    def __init__(self):
        self.location = dict() # location maps object to index
        self.queue = [[-100, 0]] #priority comes first

    #return 1 if a > b,  -1 if a < b, else 0
    #based solely on priority
    def compareIndexes(self , a , b):
        if (self.queue[a][0] > self.queue[b][0]):
            return 1
        if (self.queue[a][0] < self.queue[b][0]):
            return -1
        return 0

    #swap objects in queue index a and b
    def swapIndexes(self, a, b):
        temp = self.queue[a]
        self.queue[a] = self.queue[b]
        self.queue[b] = temp
        self.location[self.queue[a][1]] = a
        self.location[self.queue[b][1]] = b


    #Insert an element into the priority queue with a given priority
    def insert(self, priority, object):
        self.location[object] = len(self.queue)
        spot = len(self.queue)
        self.queue.append([priority, object])
        while (self.compareIndexes(spot, spot // 2) == -1): #it's a minQueue
            self.swapIndexes(spot, spot // 2) # integer division
            spot = spot // 2

    #Returns and removes the element associated with the smallest priority
    def getMin(self):
        # swap first and last, pop last, bubble down first and re map it
        temp = self.queue[1][1]
        self.swapIndexes(1, len(self.queue)-1)
        del self.location[self.queue[len(self.queue)-1][1]]
        del self.queue[len(self.queue)-1]
        spot = 1
        while ( (spot * 2)+1 < len(self.queue) ):
            if ( (self.compareIndexes(spot, spot * 2) == 1) or (self.compareIndexes(spot, (spot * 2)+1) == 1) ): #it's a minQueue
                if (self.compareIndexes(spot * 2, (spot * 2)+1) == -1):
                    self.swapIndexes(spot, spot * 2)
                    spot = spot* 2
                else:
                    self.swapIndexes(spot, (spot * 2)+1)
                    spot = (spot* 2)+1
            else:
                break
        if (spot * 2 < len(self.queue) and self.compareIndexes(spot, spot * 2) == 1):
            self.swapIndexes(spot, spot * 2)
        return temp

    #Returns the number of elements in the priority queue
    def size(self):
        return len(self.queue)-1

    def printMe(self):
        print (self.queue[1:])

    #changes the priority
    def changePriority(self, updatedPriority, element):
        spot = self.location[element]
        oldPriority = self.queue[spot][0]
        self.queue[spot][0] = updatedPriority
        if (oldPriority < updatedPriority): # move it towards end of array
            while ( (spot * 2)+1 < len(self.queue) ):
                if (self.compareIndexes(spot, spot * 2) == 1 or self.compareIndexes(spot, (spot * 2)+1) == 1): #it's a minQueue
                    if (self.compareIndexes(spot * 2, (spot * 2)+1) == -1):
                        self.swapIndexes (spot, spot*2)
                        spot = spot*2
                    else :
                        self.swapIndexes (spot, (spot * 2)+1)
                        spot = (spot*2)+1
            if (spot * 2 < len(self.queue) and self.compareIndexes(spot, spot * 2) == -1):
                self.swapIndexes(spot, spot2)
        if (oldPriority > updatedPriority): # move it towards start of array
            while (self.compareIndexes(spot, spot // 2) == -1): #it's a minQueue
                self.swapIndexes(spot, spot // 2)
                spot = spot // 2
