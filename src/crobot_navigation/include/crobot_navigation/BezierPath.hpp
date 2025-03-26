#ifndef BEZIER_PATH_HPP
#define BEZIER_PATH_HPP

#include <vector>
#include <cmath>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using Pose2D = geometry_msgs::msg::Pose2D;

class BezierPath {

    public:
        // Default Constructor //
        BezierPath();
        ~BezierPath();

        // Bezier Functions //
       
        // Generates a point on a given bezier curve given the points and t value
        Pose2D pathBezier(const std::vector<Pose2D>& points, double t, const std::vector<double>& binomialCoef);

        // Used for generating bezier curves of a higher order.
        std::vector<double> binomialCoefficients(int n);

        // Creates a vector of a Bezier curve housing 1001 reference points
        std::vector<Pose2D> setupPath(std::vector<Pose2D>& points, const std::vector<double>& binomialCoef);

        // Finds the closest t value to the robot using the vector created in setupPath()
        double closestT(std::vector<Pose2D>& points, Pose2D currentPos, double currentT, const std::vector<double>& binomialCoef);       
};

#endif