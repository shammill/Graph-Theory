/// File: graph.cpp
/// Encapsulates graph representations along with functionality to determine
/// shortest paths, and the minimum spanning tree for the graph.

#include "graph.h"
#include "disjointset.h"
#include <iostream>
#include <iomanip>

using namespace std;

/// Constructor sets the number of vertices in this Graph. Initialises the two dimensional
/// array of weights by setting all  values to INFINITY (some value larger than any
/// possible edge weight) except the diagonal of the array where the weight is set to 0.
Graph::Graph(unsigned int numVertices) {

    this->numVertices = numVertices;

    weights = new double*[numVertices];
    for(unsigned int row = 0; row < numVertices; ++row) {
        weights[row] = new double[numVertices];
		for(unsigned int column = 0; column < numVertices; ++column) {
			if (row == column) {
				weights[row][column] = 0.00;
			}
			else {
				weights[row][column] = 1000000000;
			}
		}
    }
}

/// Destructor
Graph::~Graph() {
    for(unsigned int i = 0 ; i < numVertices ; i++) {
        delete[] weights[i];
        delete[] weights;
    }
}

/// Adds pointer to Vertex to the collection of vertices for this Graph.
void Graph::addVertex(Vertex* vertex) {
   this->vertices.push_back(vertex);
}

/// Accessor  returns  a  pointer  to  the  Vertex with the identifier/index in the parameter.
Vertex* Graph::getVertex(int index) {
    return vertices[index];
}

/// Adds pointer to Edge to the edge list for this Graph. Using the source and destination
/// identifiers from the edge, sets the weight of the undirected edge in the adjacency matrix.
void Graph::addEdge(Edge* edge) {
    this->edges.push(edge);

	double weight = edge->getWeight();
	unsigned int source = edge->getSource()->getId();
	unsigned int destination = edge->getDestination()->getId();

	weights[source][destination] = weight;
	weights[destination][source] = weight;
}

/// Uses Kruskal’s algorithm to find the Minimum Spanning Tree (MST) for this Graph. Stores the
/// edges of the MST in the adjacency list of each Vertex. Returns the cost of the minimum spanning tree.
double Graph::minimumSpanningTreeCost() {
    double minCost;
    int edgeCount = 0;
    DisjointSet mst = DisjointSet(numVertices);                     // initialise DisjointSet of size N
    while((!edges.empty()) & (edgeCount < numVertices - 1)) {       // while edges not empty AND edgeCount < N – 1
        Edge* edge = edges.top();                                   // edge pq.pop()
        edges.pop();
        unsigned int source = edge->getSource()->getId();
        unsigned int destination = edge->getDestination()->getId();
        if (!mst.sameComponent(source, destination)) {               // if NOT sameComponent(edge.source, edge.destination)
                edgeCount++;                                        // increment edgeCount
                mst.join(source, destination);                      // join(edge.source, edge.destination)
                Vertex* vertex1 = getVertex(source);
                Vertex* vertex2 = getVertex(destination);
                vertex2->addAdjacency(source);                      // add source to adjacency list of destination
                vertex1->addAdjacency(destination);                 // add destination to adjacency list of current
                edges.push(edge);
                minCost = minCost + edge->getWeight();
            }
    }
    return minCost;
}

/// Determines the shortest path from the source vertex to all other vertices. Prints the length
/// of  the  path  and  the  vertex  identifiers  in  the path.
void Graph::dijkstra(unsigned int sourceId) {

    priority_queue<Vertex*, vector<Vertex*>, Vertex> vertexQueue;      /// create a priority queue of Vertex*
    for (vector<Vertex*>::iterator v = this->vertices.begin(); v != this->vertices.end(); v++) {  ///for all vertices
        Vertex* vertex = *v;
        vertex->setDiscovered(false);				/// set visited to false
		vertex->setPredecessorId(sourceId);			/// set the predecessorId to sourceId
		vertex->setMinDistance(weights[sourceId][vertex->getId()]);		/// set minimum distance to distance from source to this vertex from the adjacency matrix (weights).s
		vertexQueue.push(vertex);							/// push vertex onto priority queue
    } // end for

	while(!vertexQueue.empty()) {				/// while priority queue is not empty
        Vertex* u = vertexQueue.top();			/// queuepoll the priority queue (let this vertex be called u)
        vertexQueue.pop();
		u->setDiscovered(true);			        /// mark u as visited

		for (vector<Vertex*>::iterator i = this->vertices.begin(); i != this->vertices.end(); i++) {
            Vertex* v = *i;
            if ((weights[u->getId()][v->getId()] < 1000000000) & (!v->isDiscovered())) {
			    //cout << "U-ID: " << u->getId() << ":" << u->getMinDistance() << " -- " << "V-ID: " << v->getId() << ":" << v->getMinDistance() << endl;
				if (u->getMinDistance() + weights[u->getId()][v->getId()] <= v->getMinDistance()) {
                    //cout << "u + w < v -- I change." << endl;
					v->setMinDistance(u->getMinDistance() + weights[u->getId()][v->getId()]); 	//set v.minDistance to the new valuee
					v->setPredecessorId(u->getId());     // v->predecessorId to u.id
					vertexQueue.push(v);
				}	/// end if
            }
			//}	/// end if
        }	/// end for
    }	 /// end while

    // output the length of the shortest paths and the paths
	for (vector<Vertex*>::iterator v = this->vertices.begin(); v != this->vertices.end(); v++) {	/// for each vertex other than the source
        Vertex* vertex = *v;
        double distance = vertex->getMinDistance();		///  retrieve the minimum distance

        unsigned int predecessor = 1000000000;
        vector <unsigned int> path;

		if ((distance == 1000000000) & (!vertex->getId() == sourceId)) {				/// if minDistance == INFINITY
            cout << "NO PATH  from " << sourceId << " to ";
            cout << fixed << setw(2) << vertex->getId() << endl;
		}
        else if (!vertex->getId() == sourceId) {
            cout << "Distance from 0 to ";
            cout << fixed << setw(2) << vertex->getId() << " = ";
            cout << setprecision(2) << fixed << setw(6) << distance;
            cout << " travelling via ";

            while (predecessor != sourceId) {
                path.insert(path.begin() ,vertex->getId());
                predecessor = vertex->getPredecessorId();
                vertex = vertices[predecessor];
                if (vertex->getId() == sourceId) {
                    path.insert(path.begin(), (vertex->getId()));
                }
            }
            for (vector<unsigned int>::iterator p = path.begin(); p != path.end(); p++) {
                unsigned int node = *p;
                cout << setw(2);
                cout << node << " ";
            }
            cout << endl;
        }
    }  	/// end if
}

/// Determines the shortest path from the source vertex to all other vertices using only the adjacencies
/// in the minimum spanning tree. Prints the length of the path and the vertex identifiers in the path.
void Graph::bfs(unsigned int sourceId) {

    for (vector<Vertex*>::iterator v = this->vertices.begin(); v != this->vertices.end(); v++) {     // for each vertex in the graph
        Vertex* vertex = *v;
        vertex->setDiscovered(false);                   // set vertex discovered field to false
    }

    queue<Vertex*> vertexQueue;                                 // create an empty queue
    this->vertices[sourceId]->setDiscovered(true);              // set source vertex discovered to true
    vertexQueue.push(vertices[sourceId]);                       // push source vertex onto queue

    while (!vertexQueue.empty()) {                          // while queue is not empty
        Vertex* currentVertex = vertexQueue.front();        /// set current to front of queue (i.e. remove front)
        vertexQueue.pop();
       // cout << "Current Vertex = " << currentVertex->getId() << endl;

        //for (vector<Vertex*>::iterator v = this->vertices.begin(); v != this->vertices.end(); v++) {   /// for each vertex adjacent to current
        //Vertex* vertex = *v;

        set<unsigned int>* adjacencies = currentVertex->getAdjacencies();
            for (set<unsigned int>::iterator i = adjacencies->begin(); i != adjacencies->end(); i++) {
                unsigned int index = *i;
               // cout << currentVertex->getId() << " - i: " << index << endl;
                Vertex* adjacent = this->vertices[index];
                if (!adjacent->isDiscovered()) {                      /// if not adjacent.discovered
                    adjacent->setPredecessorId(currentVertex->getId());                   /// set adjacent.predecessor to current id
                    //cout << "I set adjacent: " << adjacent->getId() << " to have pred: " << adjacent->getPredecessorId() << endl;
                    adjacent->setDiscovered(true);                       /// set adjacent.discovered to true
                    vertexQueue.push(adjacent);                          /// push adjacent onto queue
                } //end if
            } //end for
        //}   //end for
    }   //end while

    // output the length of the shortest paths and the paths
	for (vector<Vertex*>::iterator v = this->vertices.begin(); v != this->vertices.end(); v++) {	/// for each vertex other than the source
        Vertex* vertex = *v;
        double distance = 0;		///  retrieve the minimum distance

        unsigned int predecessor = 1000000000;
        vector <unsigned int> path;

		if ((vertex->getMinDistance() == 1000000000) & (!vertex->getId() == sourceId)) {				/// if minDistance == INFINITY
            cout << "NO PATH  from " << sourceId << " to ";
            cout << fixed << setw(2) << vertex->getId() << endl;
		}
        else if (!vertex->getId() == sourceId) {
            cout << "Distance from 0 to ";
            cout << fixed << setw(2) << vertex->getId() << " = ";

            while (predecessor != sourceId) {
                path.insert(path.begin() ,vertex->getId());
                predecessor = vertex->getPredecessorId();
                distance += weights[vertex->getId()][vertex->getPredecessorId()];
                vertex = vertices[predecessor];
                if (vertex->getId() == sourceId) {
                    path.insert(path.begin(), (vertex->getId()));
                }
            }
            cout << setprecision(2) << fixed << setw(6) << distance;
            cout << " travelling via ";

            for (vector<unsigned int>::iterator p = path.begin(); p != path.end(); p++) {
                unsigned int node = *p;
                cout << setw(2);
                cout << node << " ";
            }
            cout << endl;
        }
    }
}

/// Outputs the adjacency matrix for the graph. If an edge weight is INFINITY, - should be printed instead of a number.
ostream& operator<<(ostream& out, Graph& graph) {

    for(unsigned int row = 0; row < graph.numVertices; ++row) {
		for(unsigned int column = 0; column < graph.numVertices; ++column) {

			double weight = graph.weights[row][column];
			out << setprecision(2) << fixed << setw(7);

			if (weight == 1000000000) {
				out << "-";
			}
			else {
                out << weight;
			}
		}
		out << endl;
    }
    return out;
}


