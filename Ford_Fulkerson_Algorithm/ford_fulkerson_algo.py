from typing import TypeVar

Vertex = TypeVar("Vertex")
Edge = TypeVar("Edge")
import math

class Vertex:

    def __init__(self, id):
        """
        The initialization of the vertex class containing attributes such as list of edges which are connecting the vertices,
        weight of the vertex and other boolean attributes needed to perform the dijkstra algorithm.

        Input:
            id: The id used to label a vertex
        
        Time complexity: O(1)

        """
        self.id = id
        self.edges = []
        self.visited = False
        self.capacity = math.inf
        self.discovered = False
        self.previous = None
        self.previous_edge = None



    def visited_vertex(self):
        """
        Sets the visited attribute of vertex to true if vertex is visited which is necessary for dijkstra

        Time complexity: O(1)
        """
        self.visited = True

    def discovered_vertex(self):
        """
        Sets the discovered attribute of vertex to true if vertex is discovered which is also necessary for dijkstra

        Time complexity: O(1)
        """
        self.discovered = True

    def add_edge(self, u, v, w):
        """
        Append edges to the list of edges which represents edges connecting the vertex

        Time complexity: O(1)
        """
        forward = Edge(u,v,w, 0)
        backward = Edge(v,u,0,0)
        forward.flip = backward
        backward.flip = forward
        self.edges.append(forward)
        self.edges.append(backward)
    


class Edge:

    def __init__(self, u:Vertex , v:Vertex, w, f=0):
        """
        The initialization of the edge class 

        Input:
            u: the starting vertex
            v: the end vertex
            d: direction of edge
            w: the weight of the edge
        """
        self.u = u
        self.v = v
        self.w = w
        self.f = f
        self.res = self.w - self.f
        self.flip = None


class Graph:
    """
    The constructor of graph class which takes in inputs connections, maxIn, maxOut, origin and targets to 
    create a graph of adjacency list by using the connections to initialize a graph of 3 |D| data centers
    as each vertex is split into 3. The data centers are then connected by |C| communication channels which
    are represented as edges. However, since the vertices are split into 3. The amount of communication
    channels will be squared to |C|^2.

    Input:
        connections: connections is a list of tuples (a, b, t) where a is the ID of the data centre from which the communication channel
                     departs. b is the ID of the data centre to which the communication channel arrives and
                     t is a positive integer representing the maximum throughput of that channel.
        maxIn: The limit of sum of the throughputs across all incoming communication channels from data centre i 
        maxOut: The limit of sum of the throughputs across all outgoing communication channels from data centre i 
        origin: The source of the data centre where the data to be backed up is located
        targets: targets is a list of integers such that each integer i in it indicates the location allowed for backing up data to server
    
    Time complexity: O(|D| · |C|^2)

    Aux Space Complexity: O(|D| · |C|^2)
    """
    def __init__(self, connections, maxIn, maxOut, origin, targets):
        
        self.s = origin * 3 + 1

        self.num_vertex = 0

        # Iterates through the list of connections to check for maximum possible number of vertices
        for node in connections:
            if node[0] > self.num_vertex:
                self.num_vertex = node[0]
            if node[1] > self.num_vertex:
                self.num_vertex = node[1]

        self.num_vertex += 1

        # Splitting the vertices into 3 and initializing the graph
        self.split_vertex = self.num_vertex * 3 
        self.graph = [Vertex(i) for i in range(self.split_vertex+1)]

        # Connect split vertices/data centres with edges of weight the limits maxIn and maxOut
        for i in range(self.num_vertex):
            center = i * 3 + 1
            self.graph[center-1].add_edge(self.graph[center-1], self.graph[center], maxIn[i])
            self.graph[center].add_edge(self.graph[center], self.graph[center+1], maxOut[i])

        self.sink_index = self.split_vertex 

        # Connect the targets to a super sink
        for target in targets:
            center = 3 * target + 1
            self.graph[center+1].add_edge(self.graph[center+1], self.graph[self.sink_index], maxIn[target])

        # Connect all outgoing channels to the incoming nodes of the data centres
        for node in connections:
            outgoing = node[0] * 3 + 2
            incoming = node[1] * 3 
            self.graph[outgoing].add_edge(self.graph[outgoing], self.graph[incoming], node[2])
        

    """
    A function used to reset the attributes of all vertex in the graph needed for traversal

    Time complexity: O(V) 
    """
    def reset(self):
        for vertex in self.graph:
            vertex.discovered = False
            vertex.visited = False
            vertex.previous = None
             

    """
    Breadth first search algorithm uses a queue to look for a path from source vertex to every other vertex 
    up until the supers sink. Function returns true if a path exists.

    Input:
        s: The source which is the origin passed in from the graph class
    
    Output:
        Returns true when the vertex reaches the super sink which means that there is a path. Otherwise, return False
        if there is no path.
    
    Time complexity: O(|D| · |C|^2) with O(|D|) being the number of data centres and O(|C|^2) being the number of channels

    Aux Space Complexity: O(|D| · |C|^2) as queue takes in |D| number of data centres which stores vertices connected by edges
                          of |C|^2.
    """
    def bfs_search(self, s):

        self.reset()

        #Initializing the queue
        queue = []
        source = self.graph[s]
        source.capacity = 0

        # Appending the starting vertex into the queue for traversal
        queue.append(source)
        source.visited_vertex()

        # While queue is not empty
        while len(queue) > 0:
            # Remove the left-most vertex of the queue and mark it as visited 
            u = queue.pop(0)
            u.visited_vertex()

            if u.id == self.sink_index:
                return True            
            
            # Iterates through edges of vertex u to obtain the adjacent vertex and its residual capacity
            for edge in u.edges:
                adj_vertex = edge.v
                edge.res = edge.w - edge.f
                
                # If residual capacity more than 0 and adjacent vertex is not discovered, append adjacent vertex into the queue
                # Assign the previous vertex of adjacent vertex as the one that was popped from the queue as well as the previous
                # edge of the vertex
                if edge.res > 0 and adj_vertex.discovered == False:
                    queue.append(adj_vertex)
                    adj_vertex.discovered_vertex()
                    adj_vertex.previous = u
                    adj_vertex.previous_edge = edge

        
        return False

    """
    Backtracks or reverses the path found in bfs_search from the starting to the ending vertex.

    Input:
        s: The source which is the origin passed in from the graph class
    
    Output:
        An array representing the path of the bfs_search algorithm
    
    Time complexity: O(V)
        
    Aux space complexity: O(V)
    """
    def get_AugmentingPath(self,s):
        # Initialize the vertex as the super sink
        this_vertex = self.graph[self.sink_index]
        self.reversed_path = []

        # While the vertex is not the source or origin, append the previous edge
        # of the vertex and backtrack the vertex to its parent vertex
        while this_vertex != self.graph[s]:
            
            self.reversed_path.append(this_vertex.previous_edge)
 
            this_vertex = this_vertex.previous 
        
        # pops the last element of the list which is a NoneType
        self.reversed_path.pop()

        return self.reversed_path
        

    """
    A function used to repeatedly find the minimum residual capacity of the edges of the path found 
    in bfs_search to increment and decrement the flow of forward and backward edges respectively.
    It is used to update the residual network

    Input:
        path: The path contaning edges obtained from the bfs_search algorithm

    Time complexity: O(|C|) where |C| is the number of edges in the path
    
    Aux space complexity: O|C| where |C| is the number of edges in the path
    """
    def augmentFlow(self, path):
        # Iterates through edges in a path and append their residual capacities
        # into an array
        arr = []
        for edge in path:
            arr.append(edge.res)
        
        # Find the minimum residual capacity from the array
        self.residual_capacity = arr[0]
        for i in range(len(arr)):
            if arr[i] < self.residual_capacity:
                self.residual_capacity = arr[i]

        # Update the flow of the network
        for edge in self.reversed_path:

            edge.f += self.residual_capacity
            edge.flip.f -= self.residual_capacity
        
                


"""
Ford-Fulkerson algorithm used to find the maximum flow of a network by creating a residual network
and search for a path in the network using breadth first search. If the path exists, obtain the path
and its minimum residual capacity. The minimum residual capacity found will then be added to the max
flow.

Input:
    connections: connections is a list of tuples (a, b, t) where a is the ID of the data centre from which the communication channel
                 departs. b is the ID of the data centre to which the communication channel arrives and
                 t is a positive integer representing the maximum throughput of that channel.
    maxIn: The limit of sum of the throughputs across all incoming communication channels from data centre i 
    maxOut: The limit of sum of the throughputs across all outgoing communication channels from data centre i 
    origin: The source of the data centre where the data to be backed up is located
    targets: targets is a list of integers such that each integer i in it indicates the location allowed for backing up data to server

Output:
    max_flow: The maximum flow of a network

Time complexity: O(|D| · |C|^2)

Aux Space Complexity: O(|D| · |C|^2) 
"""
def ford_fulkerson(connections, maxIn, maxOut, origin, targets):
    # Initialize flow
    max_flow = 0
    # Create residual graph
    residual = Graph(connections, maxIn, maxOut, origin, targets)
    # While there exists an augmenting path
    while residual.bfs_search(residual.s) == True:
        # Get the path
        path = residual.get_AugmentingPath(residual.s)
        # Look for the residual capacity and update the residual network
        residual.augmentFlow(path)
        # Augment the flow by incrementing equal to the residual capacity
        max_flow += residual.residual_capacity

    return max_flow

"""
A function that uses ford-fulkerson algorithm to calculate the maximum throughput of connections 
that can flow in the channels in a network of data centres from origin to the super sink

Input:
    connections: connections is a list of tuples (a, b, t) where a is the ID of the data centre from which the communication channel
                 departs. b is the ID of the data centre to which the communication channel arrives and
                 t is a positive integer representing the maximum throughput of that channel.
    maxIn: The limit of sum of the throughputs across all incoming communication channels from data centre i 
    maxOut: The limit of sum of the throughputs across all outgoing communication channels from data centre i 
    origin: The source of the data centre where the data to be backed up is located
    targets: targets is a list of integers such that each integer i in it indicates the location allowed for backing up data to server

Output:
    max_throughput: returns the maximum possible data throughput from the data centre origin to the data centres specified in targets

Time complexity: O(|D| · |C|^2)

Aux Space Complexity: O(|D| · |C|^2) 
"""
def maxThroughput(connections, maxIn, maxOut, origin, targets):

    max_throughput = ford_fulkerson(connections, maxIn, maxOut, origin, targets)

    return max_throughput