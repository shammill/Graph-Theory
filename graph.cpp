/// File: graph.cpp
/// Encapsulates graph representations along with functionality to determine
/// shortest paths, and the minimum spanning tree for the graph.

#include "graph.h"
#include "disjointset.h"
#include <iostream>
#include <iomanip>

using namespace std;

const int INFINITY = 1000000000;

/// Constructor sets the number of vertices in this Graph. Initialises the two dimensional
/// array of weights by setting all  values to INFINITY (some value larger than any
/// possible edge weight) except the diagonal of the array where the weight is set to 0.00.
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
				weights[row][column] = INFINITY;
			}
		}
    }
}

/// Destructor
Graph::~Graph() {
    for(unsigned int i = 0 ; i < numVertices ; i++) {
        delete[] weights[i];
    }
    delete[] weights;
    vertices.clear();
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
    unsigned int edgeCount = 0;
    DisjointSet mst = DisjointSet(numVertices);

    while((!edges.empty()) & (edgeCount < numVertices - 1)) {
        Edge* edge = edges.top();
        edges.pop();
        unsigned int source = edge->getSource()->getId();
        unsigned int destination = edge->getDestination()->getId();

        if (!mst.sameComponent(source, destination)) {
                edgeCount++;
                mst.join(source, destination);
                Vertex* vertex1 = getVertex(source);
                Vertex* vertex2 = getVertex(destination);
                vertex2->addAdjacency(source);
                vertex1->addAdjacency(destination);
                edges.push(edge);
                minCost = minCost + edge->getWeight();
            }
    }
    return minCost;
}

/// Determines the shortest path from the source vertex to all other vertices using dijkstras algorithm.
void Graph::dijkstra(unsigned int sourceId) {
    priority_queue<Vertex*, vector<Vertex*>, Vertex> vertexQueue;
    for (vector<Vertex*>::iterator v = this->vertices.begin(); v != this->vertices.end(); v++) {
        Vertex* vertex = *v;
        vertex->setDiscovered(false);
		vertex->setPredecessorId(sourceId);
		vertex->setMinDistance(weights[sourceId][vertex->getId()]);
		vertexQueue.push(vertex);
    }

	while(!vertexQueue.empty()) {
        Vertex* u = vertexQueue.top();
        vertexQueue.pop();
		u->setDiscovered(true);

		for (vector<Vertex*>::iterator i = this->vertices.begin(); i != this->vertices.end(); i++) {
            Vertex* v = *i;
            if ((weights[u->getId()][v->getId()] < INFINITY) & (!v->isDiscovered())) {
				if (u->getMinDistance() + weights[u->getId()][v->getId()] <= v->getMinDistance()) {
					v->setMinDistance(u->getMinDistance() + weights[u->getId()][v->getId()]);
					v->setPredecessorId(u->getId());
					vertexQueue.push(v);
				}
            }
        }
	}
	dijkstraOutput(sourceId);
}

/// Outputs the results from the dijkstras algorithm to the console.
/// Prints the length of  the  path  and  the  vertex  identifiers  in  the path.
void Graph::dijkstraOutput(unsigned int sourceId) {
	for (vector<Vertex*>::iterator v = this->vertices.begin(); v != this->vertices.end(); v++) {
        Vertex* vertex = *v;
        double distance = vertex->getMinDistance();

        unsigned int predecessor = INFINITY;
        vector <unsigned int> path;

		if ((distance == INFINITY) & (!vertex->getId() == sourceId)) {
            cout << "NO PATH  from " << sourceId << " to " << fixed << setw(2) << vertex->getId() << endl;
		}
        else if (!vertex->getId() == sourceId) {
            cout << "Distance from 0 to " << fixed << setw(2) << vertex->getId() << " = " << setprecision(2) << fixed << setw(6) << distance << " travelling via ";

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
                cout << setw(2) << node << " ";
            }
            cout << endl;
        }
    }
}

/// Determines the shortest path from the source vertex to all other vertices using only the adjacencies
/// in the minimum spanning tree.
void Graph::bfs(unsigned int sourceId) {
    for (vector<Vertex*>::iterator v = this->vertices.begin(); v != this->vertices.end(); v++) {
        Vertex* vertex = *v;
        vertex->setDiscovered(false);
    }
    queue<Vertex*> vertexQueue;
    this->vertices[sourceId]->setDiscovered(true);
    vertexQueue.push(vertices[sourceId]);

    while (!vertexQueue.empty()) {
        Vertex* currentVertex = vertexQueue.front();
        vertexQueue.pop();

        set<unsigned int>* adjacencies = currentVertex->getAdjacencies();
        for (set<unsigned int>::iterator i = adjacencies->begin(); i != adjacencies->end(); i++) {
            unsigned int index = *i;
            Vertex* adjacent = this->vertices[index];
            if (!adjacent->isDiscovered()) {
                adjacent->setPredecessorId(currentVertex->getId());
                adjacent->setDiscovered(true);
                vertexQueue.push(adjacent);
            }
        }
    }
    bfsOutput(sourceId);
}

/// Outputs the results from the bfs to the console.
/// Prints the length of the path and the vertex identifiers in the path.
void Graph::bfsOutput(unsigned int sourceId) {
	for (vector<Vertex*>::iterator v = this->vertices.begin(); v != this->vertices.end(); v++) {
        Vertex* vertex = *v;
        double distance = 0;

        unsigned int predecessor = INFINITY;
        vector <unsigned int> path;

		if ((vertex->getMinDistance() == INFINITY) & (!vertex->getId() == sourceId)) {
            cout << "NO PATH  from " << sourceId << " to " << fixed << setw(2) << vertex->getId() << endl;
		}
        else if (!vertex->getId() == sourceId) {
            cout << "Distance from 0 to " << fixed << setw(2) << vertex->getId() << " = ";

            while (predecessor != sourceId) {
                path.insert(path.begin() ,vertex->getId());
                predecessor = vertex->getPredecessorId();
                distance += weights[vertex->getId()][vertex->getPredecessorId()];
                vertex = vertices[predecessor];
                if (vertex->getId() == sourceId) {
                    path.insert(path.begin(), (vertex->getId()));
                }
            }
            cout << setprecision(2) << fixed << setw(6) << distance << " travelling via ";

            for (vector<unsigned int>::iterator p = path.begin(); p != path.end(); p++) {
                unsigned int node = *p;
                cout << setw(2) << node << " ";
            }
            cout << endl;
        }
    }
}

/// Outputs the adjacency matrix for the graph.
ostream& operator<<(ostream& out, Graph& graph) {
    for(unsigned int row = 0; row < graph.numVertices; ++row) {
		for(unsigned int column = 0; column < graph.numVertices; ++column) {

			double weight = graph.weights[row][column];
            if (column == 0) {
                out << setprecision(2) << fixed << setw(6);
            }
            else {
                out << setprecision(2) << fixed << setw(7);
            }

			if (weight == INFINITY) {
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
