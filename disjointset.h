/// File: disjointset.h
/// A data structure used to perform Union-Find operations as required
/// in Kruskal’s Minimum Cost Spanning Tree algorithm.
/// This data structure is used for very efficient look-up (Find) to determine which set an element
/// is in. It also provides a very efficient way to join (Union) two sets together.

#ifndef DISJOINTSET_H
#define DISJOINTSET_H

#include <iostream>

using namespace std;

class DisjointSet {
public:

    /// Constructor which sets the size of this DisjointSet.
    DisjointSet(int);

    /// Destructor.
    ~DisjointSet();

    /// Returns the index of the parent set of the element in the parameter.
    int find(int);

    /// Creates the union of two disjoint sets whose indexes are passed as parameters.
    void join(int, int);

    /// Returns true if the two indexes passed as parameters are in the same set.
    bool sameComponent(int, int);

private:
    int* id;
    int N;
    int* size;
};

#endif // DISJOINTSET_H
