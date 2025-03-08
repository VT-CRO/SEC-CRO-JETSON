#include <vector>
#include <cmath>

class XyhVector {
    public:

        /****************************
         * 
         *       Constructors
         * 
        *****************************/

        // Default Constructor of XyhVector
        XyhVector();

        // Paramaterized Constructor of XyhVector fed doubles
        XyhVector(double x, double y, double h);

        // Paramaterized Constructor of XyhVector fed XyhVector
        XyhVector(const XyhVector& referenceXyhVector);

        /****************************
         * 
         *       Deconstructors
         * 
        *****************************/
       ~XyhVector();

        /****************************
         * 
         *   Arithmatic Functions
         * 
        *****************************/
        XyhVector add(double x, double y, double h);
        XyhVector add(const XyhVector& referenceXyhVector);

        XyhVector sub(double x, double y, double h);
        XyhVector sub(const XyhVector& referenceXyhVector);

        XyhVector mul(double x, double y, double h);
        XyhVector mul(const XyhVector& referenceXyhVector);

        XyhVector div(double x, double y, double h);
        XyhVector div(const XyhVector& referenceXyhVector);

        /****************************
         * 
         *         Getters
         * 
        *****************************/
        double getX() const;
        double getY() const;
        double getH() const;

        /****************************
         * 
         *   Bezier Functions
         * 
        *****************************/
       
        // Generates a point on a given bezier curve given the points and t value
        XyhVector pathBezier(const std::vector<XyhVector>& points, double t, const std::vector<double>& binomialCoef);

        // Used for generating bezier curves of a higher order.
        std::vector<double> binomialCoefficients(int n);

        // Creates a vector of a Bezier curve housing 1001 reference points
        std::vector<XyhVector> setupPath(std::vector<XyhVector>& points, const std::vector<double>& binomialCoef);

        // Finds the closest t value to the robot using the vector created in setupPath()
        double closestT(std::vector<XyhVector>& points, XyhVector currentPos, double currentT, const std::vector<double>& binomialCoef);       


    private:
        double _x{};
        double _y{};
        double _h{};
};