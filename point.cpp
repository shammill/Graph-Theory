/// File: point.cpp
/// This class provides a two-dimensional representation of a
/// Cartesian point with an x co-ordinate and a y co-ordinate.
/// It has functionality to determine the distance between
/// two instances of Point objects.

#include "point.h"
#include <iostream>
#include <math.h>
#include <cmath>

using namespace std;

/// Constructor that sets the x and y coordinates for the Point object.
Point::Point(double xCoordinate, double yCoordinate) {
    this->xCoordinate = xCoordinate;
    this->yCoordinate = yCoordinate;
}

/// Returns the Euclidean distance between this Point and the Point* parameter to the function.
double Point::distanceTo (Point* point) {
    const int SQUARED = 2;
    int x1 = xCoordinate;
    int y1 = yCoordinate;
    int x2 = point->xCoordinate;	// consider removing getters? Accessing directly.
    int y2 = point->yCoordinate;

	// Euclidean Distance calculation. http://en.wikipedia.org/wiki/Euclidean_distance
    double distance = sqrt((pow((x1 - x2),SQUARED) + (pow((y1 - y2),SQUARED))));
    return distance;
}

/// Destructor.
Point::~Point() {

}

/// Produces a string representation of this Point. e.g."2 13"
ostream& operator<<(ostream& out, Point& point) {
    out << point.xCoordinate << " " << point.yCoordinate;

}
