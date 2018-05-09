/// File: graph.h
/// Encapsulates graph representations along with functionality to determine
/// shortest paths, and the minimum spanning tree for the graph.

#ifndef GRAPH_H
#define GRAPH_H
#include <iostream>
#include <queue>
#include "vertex.h"
#include "edge.h"

class Graph {
public:

    /// Constructor  sets  the  number  of  vertices  in this  Graph.  Initialises the two dimensional
    /// array  of  weights  by  setting  all  values  to INFINITY  (some  value  larger  than  any
    /// possible edge weight) except the diagonal  of the array where the weight is set to 0.
    Graph(unsigned int);

    /// Destructor
    ~Graph();

    /// Adds pointer to  Vertex  to  the  collection of vertices for this Graph.
    void addVertex(Vertex*);

    /// Accessor  returns  a  pointer  to  the  Vertex with the identifier/index in the parameter.
    Vertex* getVertex(int index);

    /// Adds pointer to Edge to the edge list for this Graph. Using the source and destination
    /// identifiers from the edge, sets the weight of the undirected edge in the adjacency matrix.
    void addEdge(Edge*);

    /// Uses Kruskal’s algorithm to find the Minimum Spanning Tree (MST) for this Graph. Stores the
    /// edges of the MST in the adjacency list of each Vertex. Returns the cost of the minimum spanning tree.
    double minimumSpanningTreeCost();

    /// Determines the shortest path from the source vertex to all other vertices. Prints the length
    /// of  the  path  and  the  vertex  identifiers  in  the path.
    void dijkstra(unsigned int);

    /// Determines the shortest path from the source vertex to all other vertices using only the adjacencies
    /// in the minimum spanning tree. Prints the length of the path and the vertex identifiers in the path.
    void bfs(unsigned int);

    /// Outputs the adjacency matrix for the graph. If an edge weight is INFINITY, - should be printed instead of a number.
    friend ostream& operator<<(ostream&, Graph&);

private:

    unsigned int numVertices;       /// Instance field for the number of vertices in the graph.
    double** weights;               /// The adjacency matrix for this graph. Two dimensional array of weights.
    priority_queue<Edge*, vector<Edge*>, Edge> edges;   /// Storage  for  edges  to  be  used  by  Kruskal’s algorithm  for  calculating  the  minimum  cost spanning tree.
    vector<Vertex*> vertices;       /// Storage for graph vertices

};

#endif // GRAPH_H
