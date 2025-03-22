#include <vector>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>


class BezierPath {
    public:
        // Default Constructor //
        BezierPath();
        ~BezierPath();

        // Bezier Functions //
       
        // Generates a point on a given bezier curve given the points and t value
        XyhVector pathBezier(const std::vector<XyhVector>& points, double t, const std::vector<double>& binomialCoef);

        // Used for generating bezier curves of a higher order.
        std::vector<double> binomialCoefficients(int n);

        // Creates a vector of a Bezier curve housing 1001 reference points
        std::vector<XyhVector> setupPath(std::vector<XyhVector>& points, const std::vector<double>& binomialCoef);

        // Finds the closest t value to the robot using the vector created in setupPath()
        double closestT(std::vector<XyhVector>& points, XyhVector currentPos, double currentT, const std::vector<double>& binomialCoef);       
};