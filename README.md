# Shortest-Paths-in-a-Network

Project: Shortest Paths in a Network

Programming Language Used: Python
Compiler Version: 3.11.6

Breakdown of the files:

graph.py: The graph.py file processes the input given from the network.txt text file using the command line argument 
and a series of user queries, using input commands. It interprets these queries to execute various 
graph-related operations, such as finding the shortest path using Dijkstra's algorithm, 
adding or deleting edges, displaying the graph, modifying vertex and edge states, 
and finding the reachable nodes in the graph using the Depth-First Search (DFS) traversal.

Breakdown of the program:

Classes:

NetworkInput Class: Class which takes the input file of the graph, which contains the following functions: 
                
                read_network_file(): function to read the data from the input file
                
                get_network(): function to return the list of lines from the input file

Edge Class: Class to define the edge between the vertex which contains the function: 
                
                edge_list(): function to return the edge_list list to store the edges of the Graph

Vertex Class: Class to define the vertex of the graph and which contains the following functions:
                
                get_all_nodes(each_line): function to return the vertex list of the Graph
                
                update_vertex(): function to return the updated vertex list whenever the status of the vertex changes
                
                get_all_edges(): function to get all the edges from source to destination from the Graph

Graph Class: Class to define the structure of the graph. Dictionary data structure is used to create the Graph structure, and this class contains the following functions: 
                
                check_node(tempNode): function to check whether the node given in the input is present in the Graph

                check_edge(tailvertex,headvertex): function to check whether the edge is present for a given node by taking the tailvertex (source vertex) and headvertex (destination vertex) as input 
                
                update_node(newNode): function to update the new node in the Graph
                
                get_updated_node(node): function to get the updated nodes in the Graph
                
                print_graph(): function to print the Graph as an adjacency list representation with the given specified format
                
                add_edge_to_initial_graph(each_lines): function to map the edges of the initial Graph by taking the adjacency list of the given vertex. Bidirectional mapping will be done, i.e. the edge will be created from source vertex to the destination vertex and vice versa.
                
                deleteedge(tailvertex, headvertex): function to delete the edge from the Graph by taking the tailvertex (source vertex) and headvertex (destination vertex) as the input  
                
                addedge_unidirection(tailvertex,headvertex,transmit_time): function to add a new edge to the Graph. Unidirectional mapping is done so the edge will be created from the tailvertex (source vertex) to the headvertex (destination vertex) and the weight (transmit_time) is added to the edge.
                
                edgedown(tailvertex,headvertex): function to perform the edgedown operation on the given edge from the tailvertex (source vertex) to the headvertex (destination vertex) which will make the edge status as 'DOWN' making it inactive for traversal
                
                edgeup(tailvertex,headvertex): function to perform the edgeup operation on the given edge from the tailvertex (source vertex) to the headvertex (destination vertex) which will make the edge status as 'UP' making it active for traversal
                
                update_status_vertex(tailvertex,status): function to update the status 
                
                vertexdown(tailvertex): function to perform the vertexdown operation on the given vertex (tailvertex) which will make the vertex status as 'DOWN' making it inactive for traversal
                
                vertexup(tailvertex): function to perform the vertexup operation on the given vertex (tailvertex) which will make the vertex status as 'UP' making it active for traversal
                
                shortest_path_dijkstra(start_node,end_node): function to perform the dijkstra's shortest path algorithm to find the path between the start_node (source vertex) and the end_node (destination vertex) which will be taken as input
                
                dfs_traversal(node,visited,reachable): function to perform the Depth First Search (DFS) traversal to get the rechable nodes from the source node by taking the node (source vertex), visited (set of vertices which are already visited), reachable (set of vertices that are reachable from the given input vertex) values as inputs
                
                get_reachable_vertices(): function to get the list of reachable vertices from the given source node
                

Data Structures design: 

The following data structures are used in the project:
For the Graph, Dictionary data structure is used. 
For storing the vertices inside the Graph, Tuple data structure is used.
For storing the nodes and tracking their updated status, List data structure is used.

What works and not works:

Error handling works and different operations performed on the Graph work for all the test cases. 
These operations include:
            Adding Unidirected Edge to the Graph
            Deleting Edge from the Graph
            Printing the Graph
            Performing the Edgedown operation on the Edge of the Graph
            Performing the Edgeup operation on the Edge of the Graph
            Performing the Vertexdown operation on the Vertex of the Graph
            Performing the Vertexup operation on the Vertex of the Graph
            Finding the Shortest Path and Shortest Distance between two nodes in the Graph using the Dijkstra's Shortest Path Algorithm
            Finding the Reachable nodes from all the nodes using the Depth-First Search (DFS) traversal

Time complexity:

Time complexity of Dijkstra's algorithm: O(V + E logV), 
where V represents the number of vertices and E represents the number of edges in the Graph.

For Reachability, first, all the vertices from the graph are sorted in an alphabetical order. 
Then, a depth-first search is recursively performed for each vertex. 
This will result in a list of neighbor vertices which are again arranged in an ascending order.

Because of the nested nature of operations, the overall time complexity for Reachability: O(V(V+E)),
where sorting the vertices will require time proportional to the number of vertices, indicating the first V in the expression. 
Next, since the DFS explores all edges and vertices, it results in (V+E) in the expression. 
This shows that the time required will increase with both the number of vertices and edges in the Graph.
