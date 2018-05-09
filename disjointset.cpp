/// File: disjointset.cpp
/// A data structure used to perform Union-Find operations as required
/// in Kruskal’s Minimum Cost Spanning Tree algorithm.
/// This data structure is used for very efficient look-up (Find) to determine which set an element
/// is in. It also provides a very efficient way to join (Union) two sets together.

#include "disjointset.h"
#include <iostream>
#include <map>

using namespace std;

/// Constructor which sets the size of this DisjointSet.
DisjointSet::DisjointSet(int N) {

    this->N = N;
    id = new int[N];
    size = new int[N];
    for (int i = 0; i < N; i++) {
        id[i] = i;
        size[i] = 1;
    }
}

/// Destructor.
DisjointSet::~DisjointSet() {

}

/// Returns the index of the parent set of the element in the parameter.
int DisjointSet::find(int i) {
    while (i != id[i]) {
        id[i] = id[id[i]];
        i = id[i];
    }
    return i;
}

/// Creates the union of two disjoint sets whose indexes are passed as parameters.
void DisjointSet::join(int p, int q) {
    int i = find(p);
    int j = find(q);
    if (size[i] < size[j]) {
        id[i] = j;
        size[j] += size[i];
    } else {
        id[j] = i;
        size[i] += size[j];
    }
}

/// Returns true if the two indexes passed as parameters are in the same set.
bool DisjointSet::sameComponent(int p, int q) {
	return find(p) == find(q);
}
