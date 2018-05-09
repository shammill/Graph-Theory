/// File: point.cpp
/// This class provides a two-dimensional representation of a
/// Cartesian point with an x co-ordinate and a y co-ordinate.
/// It has functionality to determine the distance between
/// two instances of Point objects.

#include "point.h"
#include <iostream>
#include <math.h>
#include <iomanip>

using namespace std;

/// Constructor that sets the x and y coordinates for the Point object.
Point::Point(double xCoordinate, double yCoordinate) {
    this->xCoordinate = xCoordinate;
    this->yCoordinate = yCoordinate;
}

/// Returns the Euclidean distance between this Point and the other Point*.
/// Euclidean Distance calculation. http://en.wikipedia.org/wiki/Euclidean_distance
double Point::distanceTo (Point* point) {
    double dx = xCoordinate - point->xCoordinate;
    double dy = yCoordinate - point->yCoordinate;

    double distance = sqrt((dx * dx) + (dy * dy));
    return distance;
}

/// Produces a string representation of this Point. e.g."2 13"
ostream& operator<<(ostream& out, Point& point) {
    out << setw(2) << point.xCoordinate << " " << setw(2) << point.yCoordinate;
    return out;
}
