#include "crobot_navigation/XyhVector.hpp"

// TODO: We want to move away from using the XyH data type and just using ros's built-in PoseStamped data type
//       instead. Therefore, we no longer need the class (or maybe we replace it with a BezierPath class) and 
//       we'll only need the functions relevant to bezier curves.
//       

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
 *       Deconstructors
 * 
*****************************/

XyhVector::~XyhVector() {}


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
 *   Bezier Functions
 * 
*****************************/

// Generates a point on a given bezier curve given the points and t value
XyhVector XyhVector::pathBezier(const std::vector<XyhVector>& points, double t, const std::vector<double>& binomialCoef) {
    int n = points.size() - 1;

    XyhVector target;
    
    for (int i = 0; i <= n; i++) {
        target._x = target.getX() + binomialCoef[i] * pow(1-t, n-i) * pow(t, i) * points[i].getX();
        target._y = target.getY() + binomialCoef[i] * pow(1-t, n-i) * pow(t, i) * points[i].getY();
    }
    target._h = (1-t) * points[0].getH() + t * points[n].getH();

    return target;
}

// Used for generating bezier curves of a higher order.
std::vector<double> XyhVector::binomialCoefficients(int n) {
    std::vector<double> b(n + 1);
        b[0] = 1;
        b[1] = 1;
        if(n == 1){
            return(b);
        } else {
            for(int i = 1; i < n; i ++) {
                b[i + 1] = 1;
                for(int k = i; k > 0; k --) {
                    b[k] = b[k] + b[k - 1];
                }
            }
            return(b);
        }
}

// Creates a vector of a Bezier curve housing 1001 reference points
std::vector<XyhVector> XyhVector::setupPath(std::vector<XyhVector>& points, const std::vector<double>& binomialCoef) {
    std::vector<XyhVector> reference_points;
    int index = 0;
    for (double t = 0.00; t <= 1.0; t += 0.001) {
        reference_points.push_back(pathBezier(points, t, binomialCoef)); 
        index++;
    }
    return reference_points;
}

// Finds the closest t value to the robot using the vector created in setupPath()
double XyhVector::closestT(std::vector<XyhVector>& referencePoints, XyhVector currentPos, double t, const std::vector<double>& binomialCoef) {
    XyhVector bezier = pathBezier(referencePoints, t, binomialCoef);
    double pathX = bezier.getX();
    double pathY = bezier.getY();
    double min_distance = sqrt(pow(pathX - currentPos.getX(), 2) + pow(pathY - currentPos.getY(), 2));
    double new_t = t;

    double starting_t;
    if (t - 0.05 > 0) {
        starting_t = t - 0.05;
    } else {
        starting_t = 0;
    }

    for (double i = starting_t; i < 0.05 + t && t <= 1.0; i += 0.001) {
        bezier = pathBezier(referencePoints, i, binomialCoef);
        pathX = bezier.getX();
        pathY = bezier.getY();

        double temp = sqrt(pow(pathX - currentPos.getX(), 2) + pow(pathY - currentPos.getY(), 2));
        if (temp < min_distance) {
            min_distance = temp;
            new_t = i;
        }
    }

    return new_t;
}