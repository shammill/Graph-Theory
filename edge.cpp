/// File: edge.cpp
/// This  class  encapsulates  pointers  to  the  source  and  destination  Vertex  objects  and  the
/// weight  of the  Edge. An alternative  would be to encapsulate  Vertex  identifiers  (ints)
/// rather than pointers to Vertex.

#include "edge.h"
#include <iostream>

using namespace std;

/// No argument constructor.
Edge::Edge() {
}

/// Constructor  which  sets  the  source  vertex, the  destination  vertex  and  the  weight for this Edge.
Edge::Edge(Vertex* source, Vertex* destination, double weight) {
    this->source = source;
    this->destination = destination;
    this->weight = weight;
}

/// Destructor.
Edge::~Edge() {

}

/// Returns a pointer to the source vertex.
Vertex* Edge::getSource() {
    return this->source;
}

/// Returns  a  pointer  to  the  destination vertex
Vertex* Edge::getDestination() {
    return this->destination;
}

/// Returns the weight of this Edge.
double Edge::getWeight() {
    return this->weight;
}

/// Function  operator  provides  an  ordering  for edges. Returns true if the weight of the first
/// parameter  is  greater  than  that  of  the  second paramter.
bool Edge::operator()(Edge* edge1, Edge* edge2) {
    return (edge1->getWeight() > edge2->getWeight());
}

/// Returns a string representation of this  Edge. Useful for debugging purposes.
ostream& operator<<(ostream& out, Edge& edge) {
    out << edge.getWeight();
    return out;
}
