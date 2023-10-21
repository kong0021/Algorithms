from typing import TypeVar

Vertex = TypeVar("Vertex")
Edge = TypeVar("Edge")

import math

class Graph:
    
    def __init__(self, roads, passengers):
        """
        The constructor of Graph class which takes in roads and passengers to create an adjacency list.
        The adjacency list is created by using roads to initialize a graph of size |L| key locations using
        |R| roads. This graph contains vertices and edges of carpool lanes. However, if there are passengers,
        the size of graph is doubled and contains 2|L| key locations and 2|R| roads which makes up edges 
        representing carpool and non-carpool lanes with passengers |P| which can only be at most |L|-2 which is still
        O|L|.

        Input:
            roads: A list of roads represented in tuples (a,b,c,d) which connects a starting location, a to an 
                   ending location, b with c being the total time travelling alone and d being the total time
                   travelling in carpool lane with one or more passengers.

            passengers: key locations that contains potential passengers
        
        Time complexity: O(|R|+|L|)

        Aux Space complexity: O(|R|+|L})
                   
        """
        
        self.num_vertex = 0

        # Iterates through the list of roads to check for maximum possible number of vertices
        for routes in roads:
            if routes[0] > self.num_vertex:
                self.num_vertex = routes[0]
            if routes[1] > self.num_vertex:
                self.num_vertex = routes[1]

        self.num_vertex += 1

        # Creates a graph of vertices based on maximum number of vertices obtained from iteration of list of roads.
        # If passengers are present, double size of the graph to represent carpool lane vertices and edges with time complexity O|L|
        self.graph = [Vertex(i) for i in range(self.num_vertex)]
        if len(passengers) > 0:
            self.graph += [Vertex(i) for i in range(self.num_vertex, 2 * self.num_vertex)]
        
        # Iterates through roads to add edges to all vertices which connects the key locations with time complexity O|R|
        for routes in roads:
            self.graph[routes[0]].add_edge(self.graph[routes[0]], self.graph[routes[1]], routes[2])
            if len(passengers) > 0:
                self.graph[routes[0] + self.num_vertex].add_edge(self.graph[routes[0] + self.num_vertex], self.graph[routes[1] + self.num_vertex], routes[3])

        # If there are passengers, the same vertices of non-carpool lane and carpool lanes are connected with edges with time complexity O|L|
        if len(passengers) > 0:
            for passenger in passengers:
                self.graph[passenger].add_edge(self.graph[passenger], self.graph[passenger+ self.num_vertex], 0)
            
    def dijkstra(self, source, end):
        """
        Dijkstra algorithm that uses MinHeap to find shortest path from a starting vertex to every other vertex until the end is reached.

        Input:
            source: The starting vertex where journey begins represented by an integer
            end: The destination vertex where journey ends represented by an integer
        
        Output: returns the vertex if its id is equivalent to the end which signifies the destination is reached and
                shortest path of the dijkstra algorithm is found.

        Time complexity: O(|R|log|L|), O|R| being the number of roads and O|L| is number of key locations
        
        Aux space complexity: O(|R|log|L|) as MinHeap takes in |L| number of locations which stores vertices connected by edges
                              of |R|.

        """

        start = self.graph[source]
        start.distance = 0
        # Initializing MinHeap
        self.heap = MinHeap(len(self.graph))
        # Inserting the starting vertex into MinHeap
        self.heap.insert(start)
        

        # While min heap is not empty, root node is served and is marked as visited
        while self.heap.length > 0:

            this_vertex = self.heap.serve()
            this_vertex.visited_vertex()

            # If the current vertex id is equivalent to the end value, return the current vertex
            if this_vertex.id == end or this_vertex.id == end + self.num_vertex:

                return this_vertex
            
            # Iterates through the edges connected from the served node
            for edge in this_vertex.edges:
                
                adjacent_vertex = edge.v
                edge_distance = edge.w
                
                # If the adjacent vertex connected to the served node was visited, pass 
                if adjacent_vertex.visited == True:
                    pass
                # Else, check if adjacent vertex hasn't been discovered yet, update the distances of each vertex
                # Move back from the adjacent vertex to the previous vertex
                # Set adjacent vertex as discovered and insert it into the heap.
                else:
                    if adjacent_vertex.discovered == False:
                        adjacent_vertex.distance = this_vertex.distance + edge_distance
                        adjacent_vertex.previous = this_vertex
                        adjacent_vertex.discovered_vertex()
                        self.heap.insert(adjacent_vertex)
                    # Checks if adjacent vertex is not discovered. If so, checks if the adjacent vertex distance is a longer than
                    # the total of the distance of current vertex and the weight of its edge. If so, update its distance to the shorter one
                    # Shift the adjacent vertex to the previous vertex and rise to swap the position of vertex with smaller weight with parent node
                    else:
                        if adjacent_vertex.distance > this_vertex.distance + edge_distance:
                            adjacent_vertex.distance = this_vertex.distance + edge_distance
                            adjacent_vertex.previous = this_vertex
                            self.heap.rise(self.heap.vertex_index[adjacent_vertex.id])

    def backtracking(self, start, end):
        """
        Backtracks or reverses the shortest path found in dijkstra algorithm from the starting to the ending vertex.

        Input:
            start: The starting vertex where journey begins represented by an integer
            end: The destination vertex where journey ends represented by an integer

        Output:
            A reversed array representing the shortest path of the dijkstra algorithm

        Time complexity: O(V)
        
        Aux space complexity: O(V)
        """
        # Initialize the current vertex to be the ending vertex
        this_vertex = end
        reversed_path = []

        # While current vertex is not the starting vertex
        while this_vertex != start:
            #Initialize the position of the current vertex
            position = this_vertex.id
            
            # Checks if the current position of the vertex is more than or equals the maximum number of vertices
            # This accounts for vertices with carpool lanes
            # Subtracting its position with the maximum vertex will give us its original vertex position
            if this_vertex.id >= self.num_vertex:
                position -= self.num_vertex
            # Checks if the previous vertex is not equivalent to the current vertex position
            # If so, append the current vertex position to the list
            # Repeated edges from non-carpool vertices to carpool vertices are ignored
            if this_vertex.previous.id != position:
                reversed_path.append(position)
            
            # Shift the current vertex to its previous one, to backtrack until the starting vertex is found
            this_vertex = this_vertex.previous
        
        # Append the starting vertex to the list
        reversed_path.append(this_vertex.id)

        optimal_route = []

        # List of shortest path or optimal routes is reversed by slicing
        for i in range(len(reversed_path)):
            optimal_route.append(reversed_path[len(reversed_path)-i-1])

        

        return optimal_route
        

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
        self.distance = math.inf
        self.discovered = False
        self.previous = None


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
        self.edges.append(Edge(u, v, w))
        

class Edge:
    def __init__(self, u:Vertex , v:Vertex, w):
        """
        The initialization of the edge class 

        Input:
            u: the starting vertex
            v: the end vertex
            w: the weight of the edge
        """
        self.u = u
        self.v = v
        self.w = w


class MinHeap:
    def __init__(self, size):
        """
        A Minimum Heap class which is used to find the minimum distance between vertices using dijkstra. 

        Input:
            size: the maximum size of the minimum heap
        
        Time complexity: O(1)

        Aux space complexity: O(|L|) where array stores L vertices or number of key locations
        """

        # Initializes the size of array to a fixed size depending on the maximum size input
        self.array = [None] * (size + 1)
        # Initializes an index array necessary for the mapping of positions of vertices to its index
        self.vertex_index = [0] * (size + 1)
        self.length = 0 
    
    def insert(self, element: Vertex):
        """
        Add an element to MinHeap's array
        Time Complexity: O(log V), where V is the number of elements in the MinHeap
        """
        self.length += 1
        self.array[self.length] = element
        self.vertex_index[element.id] = self.length
        self.rise(self.length)
    
    def serve(self):
        """
        Removes the last element of the heap after swapping it with the element with the minimum distance, the length is updated
        and sink is done to arrange elements to their respective positions.

        Output:
            The end vertex of the minimum heap which contains the minimum distance
        
        Time Complexity: O(log V), where V is the number of elements in the MinHeap
        """
        self.swap(1, self.length)
        last_vertex = self.array[self.length]
        self.length -= 1 
        self.sink(1)
        return last_vertex

    def swap(self, x, y):
        """
        Swap two number's position in the minheap array and also swap their indexes used to map their positions on the index array
        Time Complexity: O(1)
        """
        self.array[x], self.array[y] = self.array[y], self.array[x]
        self.vertex_index[self.array[x].id], self.vertex_index[self.array[y].id] = self.vertex_index[self.array[y].id], self.vertex_index[self.array[x].id]
    
    def rise(self, element):
        """
        Iteratively checks if the weight of current vertex is smaller than the parent vertex, if it is, swap its position with parent vertex.
        This is done until the there is not any vertices that is smaller than the parent vertex.

        Input:
            element: the position of element in the minimum heap
        
        Time Complexity: O(log V), where V is the number of elements in the MinHeap
        """
        parent = element // 2 
        while parent >= 1:
            if self.array[parent].distance > self.array[element].distance:
                self.swap(parent, element)
                element = parent 
                parent = element // 2
            else:
                break
    
    def sink(self, element):
        """
        Iteratively checks if the weight of current vertex is smaller than the parent vertex, if it is, swap its position with parent vertex.
        This is done until the there is not any vertices that is smaller than the parent vertex.

        Input:
            element: the position of element in the minimum heap

        Time Complexity: O(log V), where V is the number of elements in the MinHeap
        """
        child = 2*element 
        while child <= self.length: 
            if child < self.length and self.array[child+1].distance < self.array[child].distance:
                child += 1 
            if self.array[element].distance > self.array[child].distance:
                self.swap(element, child)
                element = child 
                child = 2*element 
            else:
                break


def optimalRoute(start: int, end: int, passengers, roads):
    """
    Finds the optimal route or shortest route from the starting vertex to the ending vertex taking into account the potential of 
    having passengers which leads to the usage of carpool lanes to shorten time and distance travelled. The graph class contains
    the necessary functions to understand more in depth on how to get this function to work.

    Input:
        start: The starting vertex where journey begins represented by an integer
        end: The destination vertex where journey ends represented by an integer
        passengers: A list of key locations containing passengers
        roads: A list of roads represented in tuples (a,b,c,d) which connects a starting location, a to an 
                ending location, b with c being the total time travelling alone and d being the total time
                travelling in carpool lane with one or more passengers.
    
    Time complexity: O(|R|log|L|)
    
    Aux space complexity: O(|R|+|L|)
    """

    # Initializes the graph
    graph = Graph(roads, passengers)
    # Perform dijkstra algorithm to find shortest path of the graph
    destination = graph.dijkstra(start, end)
    # Perform backtracking to obtain the reversed optimal route
    optimal_Route = graph.backtracking(graph.graph[start], destination)

    return optimal_Route


def main():
    # Example
    start = 54
    end = 62
    passengers = [29, 63, 22, 18, 2, 23, 48, 41, 15, 31, 13, 4, 24, 16, 27, 17, 50, 67, 37, 58, 28, 64, 35, 10, 68, 38, 59, 26, 69, 43, 44, 30, 46, 7]
    roads = [
                (31, 45, 23, 12), (48, 3, 14, 7), (58, 50, 25, 10), 
                (5, 3, 26, 23), (12, 32, 29, 3), (65, 4, 16, 16), 
                (13, 46, 14, 13), (63, 29, 10, 2), (19, 56, 30, 19), 
                (52, 47, 19, 12), (47, 52, 12, 8), (30, 42, 22, 19), 
                (46, 60, 17, 17), (54, 22, 8, 7), (19, 8, 23, 10), 
                (33, 51, 22, 5), (12, 17, 20, 5), (64, 62, 22, 18), 
                (66, 25, 28, 10), (48, 19, 23, 8), (36, 13, 22, 19), 
                (26, 48, 6, 3), (31, 30, 26, 9), (24, 29, 22, 11), 
                (23, 36, 27, 11), (59, 37, 16, 10), (60, 44, 12, 8), 
                (40, 7, 18, 1), (22, 3, 13, 12), (36, 35, 15, 15), 
                (43, 2, 23, 6), (29, 27, 27, 6), (34, 0, 17, 4), 
                (52, 50, 13, 4), (27, 23, 15, 1), (15, 10, 7, 6), 
                (36, 65, 23, 1), (41, 64, 27, 8), (45, 34, 12, 1), 
                (51, 24, 12, 10), (16, 12, 29, 7), (9, 67, 25, 24), 
                (49, 38, 16, 4), (38, 7, 10, 1), (50, 13, 23, 16), 
                (5, 33, 27, 10), (23, 42, 29, 15), (9, 2, 13, 7), 
                (59, 52, 23, 17), (59, 54, 8, 6), (1, 8, 10, 8), 
                (33, 30, 15, 2), (6, 26, 18, 6), (39, 57, 13, 12), 
                (54, 26, 13, 9), (57, 41, 4, 4), (37, 66, 16, 12), 
                (36, 9, 12, 5), (2, 68, 7, 3), (69, 28, 18, 2), 
                (44, 1, 14, 3), (48, 9, 6, 4), (17, 38, 13, 1), 
                (61, 49, 4, 4), (9, 10, 6, 3), (46, 37, 21, 8), 
                (23, 53, 21, 8), (7, 24, 28, 26), (62, 20, 22, 7), 
                (1, 18, 10, 1), (7, 41, 9, 1), (13, 18, 6, 4), 
                (25, 21, 21, 3), (1, 61, 21, 16), (49, 40, 13, 5), 
                (19, 25, 11, 10), (62, 50, 5, 5), (33, 46, 10, 9), 
                (28, 25, 14, 6), (56, 51, 6, 4), (18, 19, 15, 1), 
                (30, 9, 23, 13), (60, 21, 23, 7), (52, 37, 16, 6), 
                (50, 42, 11, 4)
                ]
    result = [54, 26, 48, 19, 56, 51, 24, 29, 27, 23, 36, 13, 46, 60, 44, 1, 61, 49, 38, 7, 41, 64, 62]

    # Your function should return the optimal route (which takes 27 minutes).
    result1 = optimalRoute(start, end, passengers, roads)
    print(result1)


if __name__ == "__main__":
    main()