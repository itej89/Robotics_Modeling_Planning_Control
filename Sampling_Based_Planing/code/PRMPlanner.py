import numpy as np
np.random.seed(3)

from AStar_Search import *
from GeometryHelper import *

#Algorithm Parametes
class Parameters:
    start = (-0.5,-0.5)
    goal = (0.5,0.5)
    Grid_min = -0.5
    Grid_max = 0.5
    N = 30 #number of samples
    k = 5 # number of closes neighbours
    robot_radius = 0


#Class Type to store an obstacle data
class OBSTACLE:
    def __init__(self, location, radius) -> None:
        #Constant Node parameters
        self.location = location
        self.radius = radius


class PRM:
    #Create objects required for PRM Algorithm
    params = Parameters()
    geomhelper = GeometryHelper()
    NODES = []
    OBSTACLES = []

    #Loads obstacles from the csv file
    #Infrates the obstacle size by robot's radius if provided in the parametes
    def LoadObstacles(self, obstacle_file):
         with open(obstacle_file, 'r') as f:
            for line in f:
                if  not line.startswith('#'):
                    meta = line.strip().split(",")
                    if len(meta) == 3:
                        # Make sure to grow the obstacle size by the radius of the robot 
                        # to turn the robot to point size
                        objObstacle = OBSTACLE((float(meta[0]), float(meta[1])),
                         (float(meta[2])/2)+self.params.robot_radius)
                        #Add each obstacle to OBSTACLES list
                        self.OBSTACLES.append(objObstacle)


    # Uniform random Samples nodes in the range [-0.5 to 0.5] (x,y) from the grid space
    def SampleGrid(self):

        #Add Start node as first node
        count = 1
        location = self.params.start
        heuristic_cost = self.geomhelper.EuclideanDistance(location, self.params.goal)
        self.NODES.append(GraphNode(count, location, heuristic_cost))

        #Sample nodes
        while len(self.NODES) < (self.params.N-1):
            intersects = False

            location = np.random.uniform(
                self.params.Grid_min,self.params.Grid_max,2)
            
            #Make sure the sampled node does not overlap with obstacle
            for obstacle in self.OBSTACLES:
                if self.geomhelper.Is_Point_Inside_Circle(obstacle.location, obstacle.radius, location):
                     intersects = True   
                     break

            if not intersects:
                count += 1
                heuristic_cost = self.geomhelper.EuclideanDistance(location, self.params.goal)

                self.NODES.append(GraphNode(count, location, heuristic_cost))
        
        #Add last node as goal node
        count += 1
        location = self.params.goal
        heuristic_cost = self.geomhelper.EuclideanDistance(location, self.params.goal)
        self.NODES.append(GraphNode(count, location, heuristic_cost))

    #Creates roadmap for the nodes samples above
    def CreateRoadmap(self):
        for current_node in self.NODES:
            distances = []

            #calculate distances to all nodes from teh given node
            for node in self.NODES:
                    if node.ID != current_node.ID:
                        distances.append(
                            (node.ID,
                            self.geomhelper.EuclideanDistance(
                            current_node.location,
                            node.location))
                        )

            distances.sort(key=lambda x: x[1])

            # Add only the nodes whose edges does not overlap with obstacles
            for edge in distances:
                #This condition limits the maximum number of neighbouring nodes
                if len(current_node.edges) < self.params.k:
                    intersects = False
                    if current_node.ID == 86 and edge[0] ==95:
                        print("break")
                    for obstacle in self.OBSTACLES:
                        intersections = self.geomhelper.circle_line_segment_intersection(obstacle.location, obstacle.radius, current_node.location, 
                                    next((x for x in self.NODES if x.ID == edge[0]), None).location)
                        if len(intersections) > 0:
                            intersects = True
                            break;

                    if not intersects:
                        if current_node.ID in next((x for x in self.NODES if x.ID == edge[0]), None).edges.keys():
                            next((x for x in self.NODES if x.ID == edge[0]), None).edges.pop(current_node.ID)
                        current_node.edges[edge[0]] = edge[1]

    #Writes nodes and edgtes to csv file
    def WriteToCsv(self, nodes, edges):
        with open(nodes, 'w') as fnodes:
            with open(edges, 'w') as fedges:
                for node in self.NODES:
                    fnodes.write(str(node.ID)+","+
                    str(node.location[0])+","+
                    str(node.location[1])+","+
                    str(node.heuristic_ctg)+"\n")

                    for edge in node.edges.keys():
                        fedges.write(str(node.ID)+","+
                        str(edge)+","+
                        str(node.edges[edge])+"\n")


if __name__ == "__main__":

     #Sample nodes and Create Roadmap using  PRM algorithm
    _PRM = PRM()
    _PRM.LoadObstacles("obstacles.csv")
    _PRM.SampleGrid()
    _PRM.CreateRoadmap()
    _PRM.WriteToCsv("nodes.csv", "edges.csv")

    # Run A* algorithm to find the path
    astart = AStar()
    astart.LoadGraphData("nodes.csv", "edges.csv")
    path = astart.FindPath(1,30)
    astart.path_to_csv(path, "path.csv")