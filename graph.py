
import numpy as np
import math
import sys
from struct import *
import heapq
from collections import defaultdict

'''Class to take the input file of the graph and perform following functions like: read_network_file(), get_network()'''
class NetworkInput:

    '''Constructor the class NetworkInput to initialize the f_path and Network variables when the object of this class is created'''
    def __init__(self, f_path):
        self.f_path = f_path
        self.Network = None

    # function to read the data from input file
    def read_network_file(self):
        try:
            with open(self.f_path, 'r') as file:
                self.Network = file.read()
        except FileNotFoundError:
            print(f"Error: File '{self.f_path}' not found.")
            sys.exit(1)
        except Exception as e:
            print(f"Error: {e}")
            sys.exit(1)

    # function to return the list of lines of the input file
    def get_network(self):
        if self.Network is not None:
            return self.Network
        else:
            print("There is no network graph given in the file") 

''' Class to define the edge between the vertex and to perform the function like: edge_list()'''
class Edge:

    # edge_list variable to store the edges of the node vertex
    edge_lits = []

    '''Constructor for the class Edge to initialize the edge_list to empty list when the object of the class Edge is created'''
    def __init__(self):
        self.edge_list = []

    # function to reeturn the edge_list list to store the edges    
    def edge_list(self):
        return self.edge_lits    

''' Class to define the vertex of the graph and to perform the following functions like:
get_all_nodes(), update_vertex(), get_all_edges()'''
class Vertex:

    '''Constructor to initialize the vertex list, edge list and status  to default status ('UP')'''
    def __init__(self):
        vertex = []
        edge = []
        status = 'UP'

    # vertex list to hold all the vertex of the graph
    vertices = []

    # all_edges to hold the all the edges of the vertex
    all_edge = []

    # Default status is 'UP' indicating all the vertex and edges are available initially
    default_status = 'UP' 

    # list to hold the graph dictionary keys with the status
    vertex_list_keys = [] 

    # function will return all the vertex list
    def get_all_nodes(self,each_line):
        for edge in each_line:
            src, dest, cost = edge.split()
            self.vertices.append(src)
            self.vertices.append(dest)
            # vertex_list will send the graph dictionary 
        vertex_list = list(np.unique(list(self.vertices)))

        for key in vertex_list:
            new_key = (key,self.default_status)
            self.vertex_list_keys.append(new_key)
            #vertex_list will send the keys with status
        return self.vertex_list_keys
    
    # function will return updated vertex list whenever the status of the vertex changes
    def update_vertex(self,newVertex,each_line):
        self.updated_vertex_list = self.get_all_node(each_line)
        new_key = (newVertex,self.default_status)
        self.updated_vertex_list.append(new_key)
        return self.updated_vertex_list
    
    # function will get all the edges from source to destination
    def get_all_edges(self,each_line):
        for edge in each_line:
            src, dest, cost = edge.split()
            self.all_edge.append((src,dest,cost))
        return self.all_edge   


'''Class graph defines the structure of the graph (Dictionary datastructure is used to cretae a Graph structure) and perform 
the following functions like: check_node(), check_node(), get_updated_node(), print_graph(), add_edge_to_initial_graph(),
deleteedge(),addedge_unidirection(),edgedown(),edgeup(),update_status_vertex(),vertexdown(),vertexup(), vertexup(), shortest_path_dijkstra(),
dfs(),get_reachable_vertices(),get_reachable_vertices()'''
class Graph:

    # Created the object of the class Vertex
    v = Vertex()
    #Created the object of the class Edge
    e = Edge()
    # This list will store all the nodes of the graph with its status
    node_with_status = []
    # The default status of the graph will be 'UP', indicating all vertices are available initially
    default_status = 'UP'
    '''Constructor of the class Graph to initialize the Graph nodes and structure of the graph'''
    def __init__(self,GraphNodes):

        self.nodes  = GraphNodes 
        self.adj_list = {}

        for node in self.nodes:
            self.adj_list[node] = [] 

    # function to check whether the node is present in the graph or not
    def check_node(self,tempNode):

        '''This function will take input value as a node and will check in our nodes list whether the node is present or not
        This function will return True if the element is not present in the list and it will return False if the node
        is present in the list.'''

        if not any(tempNode == ele[0] for ele in self.nodes):
            return True
        else:
            False

    # function to check whether the edge is present for a given node        
    def check_edge(self,tailvertex,headvertex):

        '''This function will take input values as source vertex and destination vertex and will check if their is an edge 
        between source vertex and destination vertex. This function will return True if there is an edge or will return False
        if there is no edge'''

        for var in self.nodes:
            if var[0] == tailvertex:
                x = var
        for key in self.adj_list[x]:
            if key[0] == headvertex:
                return True
                break             

    # function to add the new node 
    def update_node(self,newNode):

        '''This function will take the input as a new node and will add in our nodes list'''

        key = (newNode,self.default_status)
        self.nodes.append(key)

    # function get the updated nodes 
    def get_updated_node(self, node):

        '''This function will take the input as node and will rturn the vertex with its updated status value from the 
        node list'''

        if any(node == ele[0] for ele in self.nodes):
            return (ele for ele in self.nodes if node == ele[0])  
        
    # function to print the graph
    def print_graph(self):

        '''The print_graph will print the graph as an adjacency list representation with the given specified format'''

        # This will display the nodes if the status of the node is 'DOWN'
        for node in sorted(self.nodes):
            if node[1] == 'UP':
                print(node[0])
            elif node[1] == 'DOWN':
                print(f"{node[0]} {node[1]}")    
            for vertex, weight, status in sorted(self.adj_list[node]):
                if status == 'DOWN':
                    print("  "+f"{vertex} {weight} {status}")
                elif status == 'UP':
                    print("  " + f"{vertex} {weight}")
           

    # function to map the edges of the initial graph 
    def add_edge_to_initial_graph(self,each_lines):

        '''This function will take the input as an adjacency list of the given vertex and will map the edges of the initial
        graph. This is a bidirection mapping i.e. the edge will created from source vertex to destination vertex and destination
        vertex to source vertex'''

        all_edges = each_lines
        default_status = 'UP'
        for tailvertex, headvertex, transmit_time in all_edges:
            for key in self.adj_list:
                if key[0] == tailvertex:
                    self.adj_list[key].append((headvertex,transmit_time,default_status))
            for key in self.adj_list:
                if key[0] == headvertex:
                    self.adj_list[key].append((tailvertex,transmit_time,default_status))

    # function to delete the edge of the graph
    def deleteedge(self, tailvertex, headvertex):

        '''This function will delete the edge from the graph by taking the source and destination vertices as the input'''
        # Search for the tailvertex from the graph and assign it to v
        for t in (ele for ele in self.nodes if tailvertex == ele[0]):
            v = t 
        # Search for the headvertex from the graph and assign it to hv    
        for h in ((ele,cost,status) for ele, cost ,status in self.adj_list[v] if headvertex == ele):
            hv = h     
        self.adj_list[v].remove(hv)    

            
    # function to add a new edge in a unidirection from source to destination
    def addedge_unidirection(self,tailvertex,headvertex,transmit_time): 

        '''This function will add a new edge to the graph from source vertex to destination vertex. Unidirectional mapping is 
        done so the edge will be created from source vertex to destination vertex'''
        # This will add a new headvertex to the graph
        if not any(key[0] == tailvertex for key in self.adj_list.keys()):
            new_node = self.get_updated_node(tailvertex)

            for new in new_node:
                n = new

            self.adj_list[n] = []
        # This will add a new tailvertex to the graph
        if not any(key[0] == headvertex for key in self.adj_list.keys()):
            new_node = self.get_updated_node(headvertex)

            for new in new_node:
                n = new

            self.adj_list[n] = []  
        # Assign the value of tailvertex with its status to v
        for t in (ele for ele in self.nodes if tailvertex == ele[0]):
            v = t
  
        self.adj_list[v].append((headvertex,transmit_time,self.default_status))    

    # function to edge down the edge 
    def edgedown(self,tailvertex,headvertex):

        '''This function will take the input edge from the given source and destination and will mark that edge as DOWN
        making it inactive for traversal'''

        for t in (ele for ele in self.nodes if tailvertex == ele[0]):
            v = t   
        for var1,var in enumerate(self.adj_list[v]):
            if var[0] == headvertex and var[2] =='UP':
                Update_status = 'DOWN'
                self.adj_list[v][var1] = (var[0], var[1], Update_status)

    #function to edge Up the edge 
    def edgeup(self,tailvertex,headvertex):

        '''This function will take the input edge from the given source and destination and will mark that edge as UP
        making it active for traversal'''

        for t in (ele for ele in self.nodes if tailvertex == ele[0]):
            v = t
        for var1,var in enumerate(self.adj_list[v]):
            if var[0] == headvertex and var[2] == 'DOWN':
                Update_status = 'UP'
                self.adj_list[v][var1] = (var[0], var[1], Update_status)


    # function to update the status of vertex
    def update_status_vertex(self,tailvertex,status):

        '''This function will take the vertex with the updated status and will update the node list with the updated vertex'''

        for t in (ele for ele in self.nodes if tailvertex == ele[0]):
            v = t
        tail_vertex_ele = self.adj_list[v]
        node_new = (v[0],status)
        self.adj_list.pop(v)
        self.nodes.remove(v)
        self.nodes.append(node_new)
        self.adj_list[node_new] = tail_vertex_ele

    # function for vertex down        
    def vertexdown(self, tailvertex):

        '''This function will take the input vertex and will mark that vertex as DOWN making it inactive for traversal'''

        update_status = 'DOWN'
        for t in (ele for ele in self.nodes if tailvertex == ele[0]):
            v = t  
        if v[1] == 'UP':
            self.update_status_vertex(tailvertex,update_status) 

    #function for the vertexup logic
    def vertexup(self, tailvertex):

        '''This function will take the input vertex and will mark that vertex as UP making it available for traversal'''

        update_status = 'UP'
        for t in (ele for ele in self.nodes if tailvertex == ele[0]):
            v = t  
        if v[1] == 'DOWN':
            self.update_status_vertex(tailvertex,update_status) 


    #function to find the shortest path        
    def shortest_path_dijkstra(self,start_node,end_node):

        '''This function will perform the Dijkstra's Shortest Path Algorithm on the given Graph 
        and find the shortest path and shortest distance from the source vertex (start_node) 
        to the destination vertex (end_node). Here Priority queue datastructure is used to perform the operation'''
        # This graph will contains all the nodes and edges which have their status as 'UP'
        new_graph = {}
        active_status = 'UP'
        inactive_status = 'DOWN'
        # This list will store all the nodes which have their status = "DOWN"
        inactive_nodes = []

        '''The shortest path operation will perfor on updated graph wiht the updated status values of each nodes'''

        # logic for the updated graph, This graph will have tailvertex: {tailvertex: transmit_time} representation
        for keyValue in self.adj_list.keys():
            if keyValue[1] == inactive_status:
                inactive_nodes.append(keyValue[0])

        for key, values in self.adj_list.items():
            source = key
            if source not in new_graph:
                if source[1] == active_status:
                    new_graph[source[0]] = {}

            if source[1] is not inactive_status:    
                for target, weight, direction in values:
                    if direction != inactive_status and target not in inactive_nodes:
                        new_graph[source[0]][target] = float(weight)

        '''The distance dict intially will mark each headvertex transmit time as Infinity value except for the starting vertex'''   

        Totaldistances = {dist: sys.float_info.max for dist in new_graph}
        # Totaldistance will be 0 initially
        Totaldistances[start_node] = 0
        previous_node = {prev: None for prev in new_graph}
        # priority Queue is a list of tuple is initialize for the starting vertex, where initial distance will be zero
        priorityQueue = [(0, start_node)] 
        # while there are elements in the priority queue
        while priorityQueue:
            # This will pop the priority item from the priorityQueue list
            current_distance, current_node = heapq.heappop(priorityQueue)

            if current_node == end_node:
                path = []
                while current_node:
                    path.append(current_node)
                    current_node = previous_node[current_node]
                    # Return distance and path
                return Totaldistances[end_node], path[::-1]  
            # checking if the current distance is greater than the total calculated distance
            if current_distance > Totaldistances[current_node]:
                continue
            # iterate through current node of the graph and calculate and update the Total distance
            for neigh, transmit_time in new_graph[current_node].items():
                dist = current_distance + transmit_time
                if dist < Totaldistances[neigh]:
                    Totaldistances[neigh] = dist
                    previous_node[neigh] = current_node
                    heapq.heappush(priorityQueue, (dist, neigh))    
        # This return statement will return the maximum infinite value and an empty list 
        return sys.float_info.max, []
    
    #function to perform DFS traversal to get the rechable nodes from source
    def dfs_traversal(self,node, visited, reachable):
        # if the node has been visited, return
        if node in visited:     
            return
        
        reachable.add(node)
        visited.add(node)
        node_tuple = None 

        # Iterate though all the items in the Graph
        for k,v in self.adj_list.items():
            if k[0] == node:
                node_tuple = k
                break

        # traverse through the neighbors of the current node in the Graph
        for neighbor, weight, status in self.adj_list[node_tuple]:
            if status == 'DOWN':
                continue
            if neighbor not in visited and neighbor: 
          
                self.dfs_traversal(neighbor, visited, reachable)

    # function to print all rechable nodes from start to end
    def get_reachable_vertices(self):

        reachable_vertices = defaultdict(set)
        down_verts = set() # created empty set to hold the vertices whose status is DOWN

        # iterate through all the nodes in the Graph
        for src in self.adj_list: 
            if src[1] == 'DOWN':
                down_verts.add(src[0])
                continue

            visited = set() 
            reachable = set()    
            self.dfs_traversal(src[0], visited, reachable)

            reachable -= down_verts # Remove vertices that are 'DOWN' from reachable set
            reachable.discard(src[0]) # Remove the source node from the reachable set

            # Store reachable vertices if the source node is not 'DOWN'
            if src[0] not in down_verts:            
                reachable_vertices[src[0]] = reachable

        # display the reachable vertices for each node in the Graph
        for node,reachable in reachable_vertices.items():
            print(node)
            # print reachable vertices in a sorted order
            for r in sorted(reachable):
                if r not in down_verts:
                    print(f" {r}")

                               
'''The main function'''

if __name__ == "__main__":
    vertex_list =[]
    # the length of command line should be less than or equal to 3
    if len(sys.argv) <= 3:
        f_path = sys.argv[1]
        f_input = NetworkInput(f_path)
        
        f_input.read_network_file()
        network_path =f_input.get_network()

    else:
        print("Invalid number of arguments :\n")

    lines = network_path.split('\n')
    #created a vertex class instance 
    vertex = Vertex()
    all_nodes = vertex.get_all_nodes(lines)
    #create a graph class instance
    graph = Graph(all_nodes)
    data = vertex.get_all_edges(lines)
    #This function call will initialize graph structure with all the edges and vertices
    graph.add_edge_to_initial_graph(data)
    '''This lines of code will execute try and catch if any unwanted exception occured then it will show us the exception message
    the While True loop will continue to take command untill we give command quit'''
    try:
        while True:

            '''The inputQuery will take the input command from the console and will split it by (space " "), the inputQuery[0]
            will indicate which operation to execute'''

            inputQuery = input("Input command:")
            takeCommand = inputQuery.split(" ")

            # if the input commadn is print then it will print the graph and will display the status of edge/vertex if status is DOWN
            if takeCommand[0] == 'print':
                graph.print_graph()

            # if the input operation is deleteedge then it will perform the deleteedge opetration, which will delete the edge
            elif takeCommand[0] == 'deleteedge':
                srcVertex = takeCommand[1]
                destVertex = takeCommand[2]
                graph.deleteedge(srcVertex,destVertex)

            # if the input command is addedge then it will perform the addedge operation
            elif takeCommand[0] == 'addedge':
                srcVertex = takeCommand[1]
                destVertex = takeCommand[2]
                transmit_time = takeCommand[3]
                if graph.check_node(srcVertex)== None and graph.check_node(destVertex) == None:
                    check_value = graph.check_edge(srcVertex,destVertex)
                    if check_value:
                        graph.deleteedge(srcVertex,destVertex)
                    graph.addedge_unidirection(srcVertex,destVertex,transmit_time)
                else:
                    if graph.check_node(srcVertex):
                        graph.update_node(srcVertex)
                    elif graph.check_node(destVertex):
                        graph.update_node(destVertex)
                    graph.addedge_unidirection(srcVertex,destVertex,transmit_time)

            # if the input command is edgedown then it will perfrom the edgedown operation
            elif takeCommand[0] == 'edgedown':
                 srcVertex = takeCommand[1]
                 destVertex = takeCommand[2]
                 graph.edgedown(srcVertex,destVertex)

            # if the input command is edgeup then it will perform edgeup operation
            elif takeCommand[0] == 'edgeup':
                 srcVertex = takeCommand[1]
                 destVertex = takeCommand[2]
                 graph.edgeup(srcVertex,destVertex)  

            # if the input command is vertexdown then it will perform vertexdown operation
            elif takeCommand[0] == 'vertexdown':
                 srcVertex = takeCommand[1]
                 graph.vertexdown(srcVertex)

            # if the input command is vertexup it will perform the vertexup operation
            elif takeCommand[0] == 'vertexup':
                 srcVertex = takeCommand[1]
                 graph.vertexup(srcVertex)

            # if the input command is path then it will perform the shortest path from source to destination
            elif takeCommand[0] == 'path':
                 start_node = takeCommand[1]
                 end_node = takeCommand[2] 
                 shortDistance, shortPath = graph.shortest_path_dijkstra(start_node,end_node)

                 if shortDistance != float('inf'):
                    val = (' '.join(map(str, shortPath))+" "+str(round(shortDistance,2)))
                    print(val)
                 else:
                    print(f"No path found between {start_node} and {end_node}")

            # if the input command is reachable then it will perform the reachable operations
            elif takeCommand[0] == 'reachable':
                graph.get_reachable_vertices()

            # Exit condition for the while loop
            elif takeCommand[0] == "quit":
                exit() 
            # for the invalid argument passed     
            else:
                print("Plase enter valid command")  

    # Whenever there is an expection it will throw the error in the following code block                
    except Exception as exception:
        print("Exception occurred!",exception)            
