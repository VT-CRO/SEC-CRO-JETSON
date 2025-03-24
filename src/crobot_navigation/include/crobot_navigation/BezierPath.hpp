#ifndef BEZIER_PATH_HPP
#define BEZIER_PATH_HPP

#include <vector>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using PoseStamped = geometry_msgs::msg::PoseStamped;

class BezierPath {

    public:
        // Default Constructor //
        BezierPath();
        ~BezierPath();

        // Bezier Functions //
       
        // Generates a point on a given bezier curve given the points and t value
        PoseStamped pathBezier(const std::vector<PoseStamped>& points, double t, const std::vector<double>& binomialCoef);

        // Used for generating bezier curves of a higher order.
        std::vector<double> binomialCoefficients(int n);

        // Creates a vector of a Bezier curve housing 1001 reference points
        std::vector<PoseStamped> setupPath(std::vector<PoseStamped>& points, const std::vector<double>& binomialCoef);

        // Finds the closest t value to the robot using the vector created in setupPath()
        double closestT(std::vector<PoseStamped>& points, PoseStamped currentPos, double currentT, const std::vector<double>& binomialCoef);       
};

#endif