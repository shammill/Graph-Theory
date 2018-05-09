/**
* File: point.h
* This class provides a two-dimensional representation of a
* Cartesian point with an x co-ordinate and a y co-ordinate.
* It has functionality to determine the distance between
* two instances of Point objects.
*/

#ifndef POINT_H
#define POINT_H
#include <iostream>

using namespace std;

class Point {
public:
   /**
    * Constructor that sets the x and y coordinates for the Point object.
    */
    Point(double, double);

   /**
    * Returns the Euclidean distance between this Point and the Point* parameter to the function.
    */
    double distanceTo(Point*);

   /**
    * Produces a string representation of this Point. e.g."2 13".
    */
    friend ostream& operator<<(ostream&, Point&);


private:
   /**
    * X and Y coordinate variables for our Point.
    */
    double xCoordinate;
    double yCoordinate;
};

#endif // POINT_H
