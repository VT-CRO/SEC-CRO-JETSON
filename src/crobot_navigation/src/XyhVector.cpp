#include "crobot_navigation/XyhVector.hpp"
using namespace std;

/****************************
 * 
 *       Constructors
 * 
*****************************/

// Default Constructor
XyhVector::XyhVector() {
    _x = 0;
    _y = 0;
    _h = 0;
}

// deconstructor
XyhVector::~XyhVector() {
}

// Paramaterized constructor (feed in set values, clamps heading)
XyhVector::XyhVector(double x, double y, double h) {
    _x = x;
    _y = y;

    // Clamp our heading between 180 and -180 degrees
    h = fmod(h + 180.0, 360.0);  // Shift by 180 to bring range to 0-360
    if (h < 0) {
        h += 360.0;  // Handle negative values
    }
    h -= 180.0;  // Shift back to -180 to 180 range
    _h = h;
}

// Paramaterized constructor (feed in another XyhVector)
XyhVector::XyhVector(const XyhVector& referenceXyhVector) {
    _x = referenceXyhVector.getX();
    _y = referenceXyhVector.getY();
    _h = referenceXyhVector.getH();
}

/****************************
 * 
 *   Arithmatic Functions
 * 
*****************************/

// Add three doubles to our current vector
XyhVector XyhVector::add(double x, double y, double h) {
    _x = _x + x;
    _y = _y + y;

    _h = _h + h;

    // Clamp our heading between 180 and -180 degrees
    _h = fmod(_h + 180.0, 360.0);  // Shift by 180 to bring range to 0-360
    if (_h < 0) {
        _h += 360.0;  // Handle negative values
    }
    _h -= 180.0;  // Shift back to -180 to 180 range
}

XyhVector XyhVector::add(const XyhVector& referenceXyhVector) {
    _x = _x + referenceXyhVector.getX();
    _y = _y + referenceXyhVector.getY();
    _h = _h + referenceXyhVector.getH();

    // Clamp our heading between 180 and -180 degrees
    _h = fmod(_h + 180.0, 360.0);  // Shift by 180 to bring range to 0-360
    if (_h < 0) {
        _h += 360.0;  // Handle negative values
    }
    _h -= 180.0;  // Shift back to -180 to 180 range
}

XyhVector XyhVector::sub(double x, double y, double h) {
    _x = _x - x;
    _y = _y - y;

    _h = _h - h;

    // Clamp our heading between 180 and -180 degrees
    _h = fmod(_h + 180.0, 360.0);  // Shift by 180 to bring range to 0-360
    if (_h < 0) {
        _h += 360.0;  // Handle negative values
    }
    _h -= 180.0;  // Shift back to -180 to 180 range
}

XyhVector XyhVector::sub(const XyhVector& referenceXyhVector) {
    _x = _x - referenceXyhVector.getX();
    _y = _y - referenceXyhVector.getY();
    _h = _h - referenceXyhVector.getH();

    // Clamp our heading between 180 and -180 degrees
    _h = fmod(_h + 180.0, 360.0);  // Shift by 180 to bring range to 0-360
    if (_h < 0) {
        _h += 360.0;  // Handle negative values
    }
    _h -= 180.0;  // Shift back to -180 to 180 range
}

XyhVector XyhVector::mul(double x, double y, double h) {
    _x = _x * x;
    _y = _y * y;

    _h = _h * h;

    // Clamp our heading between 180 and -180 degrees
    _h = fmod(_h + 180.0, 360.0);  // Shift by 180 to bring range to 0-360
    if (_h < 0) {
        _h += 360.0;  // Handle negative values
    }
    _h -= 180.0;  // Shift back to -180 to 180 range
}

XyhVector XyhVector::mul(const XyhVector& referenceXyhVector) {
    _x = _x * referenceXyhVector.getX();
    _y = _y * referenceXyhVector.getY();
    _h = _h * referenceXyhVector.getH();

    // Clamp our heading between 180 and -180 degrees
    _h = fmod(_h + 180.0, 360.0);  // Shift by 180 to bring range to 0-360
    if (_h < 0) {
        _h += 360.0;  // Handle negative values
    }
    _h -= 180.0;  // Shift back to -180 to 180 range
}

XyhVector XyhVector::div(double x, double y, double h) {
    _x = _x / x;
    _y = _y / y;

    _h = _h / h;

    // Clamp our heading between 180 and -180 degrees
    _h = fmod(_h + 180.0, 360.0);  // Shift by 180 to bring range to 0-360
    if (_h < 0) {
        _h += 360.0;  // Handle negative values
    }
    _h -= 180.0;  // Shift back to -180 to 180 range
}

XyhVector XyhVector::div(const XyhVector& referenceXyhVector) {
    _x = _x / referenceXyhVector.getX();
    _y = _y / referenceXyhVector.getY();
    _h = _h / referenceXyhVector.getH();

    // Clamp our heading between 180 and -180 degrees
    _h = fmod(_h + 180.0, 360.0);  // Shift by 180 to bring range to 0-360
    if (_h < 0) {
        _h += 360.0;  // Handle negative values
    }
    _h -= 180.0;  // Shift back to -180 to 180 range
}


/****************************
 * 
 *         Getters
 * 
*****************************/
double XyhVector::getX() const {return _x;}
double XyhVector::getY() const {return _y;}
double XyhVector::getH() const {return _h;}

/****************************
 * 
 *         Bezier Functions
 * 
*****************************/

// Generates a point on a given bezier curve given the points and t value
XyhVector XyhVector::pathBezier(const std::vector<XyhVector>& points, double t, const std::vector<double>& binomialCoef) {
    // to be implemented by res
}

// Used for generating bezier curves of a higher order.
std::vector<double> XyhVector::binomialCoefficients(int n) {
    // to be implemented by res
}

// Creates a vector of a Bezier curve housing 1001 reference points
std::vector<XyhVector> XyhVector::setupPath(std::vector<XyhVector>& points, const std::vector<double>& binomialCoef) {
    std::vector<XyhVector> reference_points;
    int index = 0;
    for (double t = 0.00; t < 1.0; i += 0.001) {
        reference_points.push_back(pathBezier(points, t, binomialCoef)); 
        index++;
    }
    return reference_points;
}

// Finds the closest t value to the robot using the vector created in setupPath()
double XyhVector::closestT(std::vector<XyhVector>& referencePath, XyhVector currentPos, double t) {
    double min_distance = sqrt(pow(getX(t) - currentPos, 2) + pow(getY(t) - currentPos, 2));
    new_t = t;

    for (double i = t; i < 0.05 + t && t < 1.0; i += 0.001) {
        temp = sqrt(pow(getX(i) - currentPos, 2) + pow(getY(i) - currentPos, 2));
        if (temp < min_distance) {
            min_distance = temp;
            new_t = i;
        }
    }

    return new_t;
}

XyhVector::double getX(double t) {
    // TO DO
    return 0;
}
XyhVector::double getY(double t) {
    // TO DO
    return 0;
}