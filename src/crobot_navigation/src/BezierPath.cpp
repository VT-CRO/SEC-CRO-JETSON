#include "crobot_navigation/BezierPath.hpp"

// Default Constructor
BezierPath::BezierPath() {
}

BezierPath::~BezierPath() {}

// Bezier Functions

// Generates a point on a given bezier curve given the points and t value
PoseStamped BezierPath::pathBezier(const std::vector<PoseStamped>& points, double t, const std::vector<double>& binomialCoef) {
    int n = points.size() - 1;

    PoseStamped target;
    
    for (int i = 0; i <= n; i++) {
        target._x = target.pose.position.x + binomialCoef[i] * pow(1-t, n-i) * pow(t, i) * points[i].pose.position.x;
        target._y = target.pose.position.y + binomialCoef[i] * pow(1-t, n-i) * pow(t, i) * points[i].pose.position.y;
    }
    target._h = (1-t) * points[0].getH() + t * points[n].getH();

    return target;
}

// Used for generating bezier curves of a higher order.
std::vector<double> BezierPath::binomialCoefficients(int n) {
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
std::vector<PoseStamped> BezierPath::setupPath(std::vector<PoseStamped>& points, const std::vector<double>& binomialCoef) {
    std::vector<PoseStamped> reference_points;
    int index = 0;
    for (double t = 0.00; t <= 1.0; t += 0.001) {
        reference_points.push_back(pathBezier(points, t, binomialCoef)); 
        index++;
    }
    return reference_points;
}

// Finds the closest t value to the robot using the vector created in setupPath()
double BezierPath::closestT(std::vector<PoseStamped>& referencePoints, PoseStamped currentPos, double t, const std::vector<double>& binomialCoef) {
    PoseStamped bezier = pathBezier(referencePoints, t, binomialCoef);
    double pathX = bezier.pose.position.x;
    double pathY = bezier.pose.position.y;
    double min_distance = sqrt(pow(pathX - currentPos.pose.position.x, 2) + pow(pathY - currentPos.pose.position.y, 2));
    double new_t = t;

    double starting_t;
    if (t - 0.05 > 0) {
        starting_t = t - 0.05;
    } else {
        starting_t = 0;
    }

    for (double i = starting_t; i < 0.05 + t && t <= 1.0; i += 0.001) {
        bezier = pathBezier(referencePoints, i, binomialCoef);
        pathX = bezier.pose.position.x;
        pathY = bezier.pose.position.y;

        double temp = sqrt(pow(pathX - currentPos.pose.position.x, 2) + pow(pathY - currentPos.pose.position.y, 2));
        if (temp < min_distance) {
            min_distance = temp;
            new_t = i;
        }
    }

    return new_t;
}