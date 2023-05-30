from cmath import inf
from tkinter import CURRENT
from matplotlib.pyplot import cla
import numpy as np
import json

import csv

#Data type for a single node
class GraphNode:
    def __init__(self, ID, location, heuristic_cost_to_go) -> None:
        #Constant Node parameters
        self.ID = ID
        self.location = location
        self.heuristic_ctg = heuristic_cost_to_go
        self.edges = {}

        #Search related node parameters
        self.PAST_COST = 0
        self.EST_TOTAL_COST = 0
        self.PARENT_NODE = None
        

class AStar:
    #Contains list of all nodes(as GraphNode type) read from CSV
    NODES = []

    #A* Search related data structures
    OPEN = []
    CLOSE = []
    CURRENT_NODE = None


    def LoadGraphData(self, nodes_file, edges_file):

        # reads nodes.csv file and creates a list of nodes
        f = open(nodes_file, 'r')
        for line in f:
            if  not line.startswith('#'):
                meta = line.strip().split(",")
                if len(meta) == 4:
                    objNode = GraphNode(float(meta[0]), (float(meta[1]), float(meta[2])), float(meta[3]))
                    self.NODES.append(objNode)
        f.close()

        
        # reads edges.csv file and creates a all the relevent edges for each node
        f = open(edges_file, 'r')
        for line in f:
            if  not line.startswith('#'):
                meta = line.strip().split(",")
                if len(meta) == 3:
                    #Same edge Information needs to be updated in the two nodes connected by the edge

                    #Add Edge ID2 to ID1's node
                    objNode = next((x for x in self.NODES if x.ID == float(meta[0])), None)
                    if objNode != None:
                        objNode.edges[float(meta[1])] = float(meta[2])
                    
                    #Add Edge ID1 to ID2's node
                    objNode = next((x for x in self.NODES if x.ID == float(meta[1])), None)
                    if objNode != None:
                        objNode.edges[float(meta[0])] = float(meta[2])
        f.close()

    def InitializeStructures(self, start, goal):
        #Add start node to OPEN list
        self.OPEN.append(next((x for x in self.NODES if x.ID == start), None))

        #Initialize Node Parametes for A* search
        for node in self.NODES:
            if node.ID == start:
                node.PAST_COST = 0
            else:
                node.PAST_COST = inf

            node.EST_TOTAL_COST = node.PAST_COST + node.heuristic_ctg
            node.PARENT_NODE = None

    def FindPath(self, start, goal):
        PATH = []

        self.InitializeStructures(start, goal)

        while len(self.OPEN) > 0:
            #Sort the OPEN list as per the estimated total cost in ascending order
            self.OPEN.sort(key=lambda x: x.EST_TOTAL_COST)

            #Load the first node fronm OPEN List
            self.CURRENT_NODE = self.OPEN[0]
            #Delete first node from the OPEN List 
            self.OPEN  = self.OPEN[1:]
            #Add first node from the OPEN List tot he CLOSE LIST
            self.CLOSE.append(self.CURRENT_NODE)

            #return if goal node is reached
            if self.CURRENT_NODE.ID == goal:
                break
            
            #ITerate through neighbouring nodes of current node and estimate their costs
            for nbr in self.CURRENT_NODE.edges.keys():
                if next((x for x in self.CLOSE if x.ID == nbr), None) == None:
                    tentative_past_cost = self.CURRENT_NODE.PAST_COST + self.CURRENT_NODE.edges[nbr]
                    nbr_node = next((x for x in self.NODES if x.ID == nbr), None)
                    if tentative_past_cost < nbr_node.PAST_COST:
                        nbr_node.PAST_COST = tentative_past_cost
                        nbr_node.PARENT_NODE = self.CURRENT_NODE.ID
                        nbr_node.EST_TOTAL_COST = nbr_node.PAST_COST + nbr_node.heuristic_ctg
                        self.OPEN.append(nbr_node)


        #Loop backwards from current node to the first node to creat the path
        PATH.append(self.CURRENT_NODE.ID)
        while self.CURRENT_NODE.PARENT_NODE != None:
            self.CURRENT_NODE = next((x for x in self.NODES if x.ID == self.CURRENT_NODE.PARENT_NODE), None)
            PATH.append(self.CURRENT_NODE.ID)

        PATH.reverse()

        return PATH


    def path_to_csv(self, path, file_name):
        with open(file_name, 'w') as f:
            write = csv.writer(f)
            write.writerow(path)




if __name__ == "__main__":
    astart = AStar()
    astart.LoadGraphData("nodes.csv", "edges.csv")
    path = astart.FindPath(1,12)
    astart.path_to_csv(path, "path.csv")
