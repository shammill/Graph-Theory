/// File: vertex.cpp
/// This class encapsulates the identifier of a Vertex object and a collection of Vertex
/// objects in the minimum spanning tree of the graph which are adjacent to this Vertex.
/// information required by Dijkstra’s algorithm, Kruskal’s algorithm and the Breadth First Search algorithm.

#include "vertex.h"
#include <iostream>

using namespace std;

/// No argument constructor.
Vertex::Vertex() {

}

/// Constructor which sets the vertex identifier (useful for indexing into collections of vertices).
Vertex::Vertex(unsigned int identifier) {
    this->identifier = identifier;
}

/// Destructor.
Vertex::~Vertex() {
    adjacencies.clear();
}

/// Accessor for this vertex’ identifier
unsigned int Vertex::getId() {
    return this->identifier;
}

/// Adds a Vertex’ identifier to this vertex’ adjacency list.
/// Used by the Graph::Minimum Spanning Tree method
void Vertex::addAdjacency(unsigned int indentifier) {
    adjacencies.insert(indentifier);
}

/// Returns a collection of int, being the vertices adjacent to this Vertex.
/// Used by the Graph::Breadth First Search method
set<unsigned int>* Vertex::getAdjacencies() {
    return &adjacencies;
}

/// Mutator for the visited field
void Vertex::setDiscovered(bool discovered) {
    this->discovered = discovered;
}

/// Accessor for the visitor field
bool Vertex::isDiscovered() {
    return this->discovered;
}

/// Mutator for the predecessorId field
void Vertex::setPredecessorId(unsigned int predecessorId) {
    this->predecessorId = predecessorId;
}

/// Accessor for the predecessorId field
unsigned int Vertex::getPredecessorId() {
    return this->predecessorId;
}

/// Mutator for the minDistance field
void Vertex::setMinDistance(double minDistance) {
    this->minDistance = minDistance;
}

/// Accessor for the minDistance field
double Vertex::getMinDistance() {
    return this->minDistance;
}

/// Function operator implementation to provide an ordering for two Vertex instances.
/// Returns true if the minDistance of the first parameter is greater than that of the second parameter.
bool Vertex::operator()(Vertex* vertex1, Vertex* vertex2) {
    return (vertex2->getMinDistance() < vertex1->getMinDistance());
}

/// Provides a string representation of this object. Useful for debugging.
ostream& operator<<(ostream& out, Vertex& vertex) {
    out << vertex.getId();
    return out;
}

