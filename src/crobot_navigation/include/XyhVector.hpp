#include <vector>
#include <cmath>
#include <iostream>

class XyhVector {
    public:
        // Default Constructor of XyhVector
        XyhVector();

        // Deconstrucor
        ~XyhVector();

        // Paramaterized Constructor of XyhVector fed doubles
        XyhVector(double x, double y, double h);

        // Paramaterized Constructor of XyhVector fed XyhVector
        XyhVector(const XyhVector& referenceXyhVector);

        // arithmetic
        XyhVector add(double x, double y, double h);
        XyhVector add(const XyhVector& referenceXyhVector);

        XyhVector sub(double x, double y, double h);
        XyhVector sub(const XyhVector& referenceXyhVector);

        XyhVector mul(double x, double y, double h);
        XyhVector mul(const XyhVector& referenceXyhVector);

        XyhVector div(double x, double y, double h);
        XyhVector div(const XyhVector& referenceXyhVector);

        // getter methods
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
        double closestT(std::vector<XyhVector>& referencePath, XyhVector currentPos, double t, const std::vector<double>& binomialCoef);       

        // helper functions
        double getX(double t);
        double getY(double t);


    private:
        double _x{};
        double _y{};
        double _h{};
};