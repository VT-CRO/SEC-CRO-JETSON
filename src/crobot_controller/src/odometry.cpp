#include "crobot_controller/odometry.hpp"

namespace crobot_controller
{
Odometry::Odometry()
:   _x(0.0),
    _y(0.0),
    _heading(0.0),

    _PI(2.0*acos(0.0)),

    _R(1.6),
    _N(4096),
    _B(0.0),
    _L(22.698711),
    _CM_PER_TICK((2.0 * _PI * _R)/_N),

    _currentLeftPosition(0.0),
    _currentRightPosition(0.0),
    _currentAuxPosition(0.0),

    _prevLeftPosition(0.0),
    _prevRightPosition(0.0),
    _prevAuxPosition(0.0)
    
{
    
}

// Main function for dead wheel odometry
// Call this whenever we want to update the position
// Note, 
bool Odometry::updatePos(long leftTicks, long rightTicks, long auxTicks)
{

    // updates our old positions using values from the previous call
    _prevLeftPosition = _currentLeftPosition;
    _prevRightPosition = _currentRightPosition;
    _prevAuxPosition = _currentAuxPosition;

    // sets our current position equal to the tick values the encoders now read
    _currentLeftPosition = leftTicks;
    _currentRightPosition = rightTicks;
    _currentAuxPosition = auxTicks;

    // gets the difference in ticks between the current and previous values
    long dn1 = _currentLeftPosition - _prevLeftPosition;
    long dn2 = _currentRightPosition - _prevRightPosition;
    long dn3 = _currentAuxPosition - _prevAuxPosition;

    // calculates the change in heading, x, and y
    double dtheta = _CM_PER_TICK * (dn2-dn1) / _L;
    double dx = _CM_PER_TICK * (dn1 + dn2) / 2.0;
    double dy = _CM_PER_TICK * (dn3 - (dn2 - dn1) * _B / _L);

    // updates the position and heading
    double theta = _heading * (dtheta / 2.0);
    _x += dx * cos(theta) - dy * sin(theta);
    _y += dx * sin(theta) + dy * cos(theta);
    _heading += dtheta;

    // Normalize the heading. This constrains it to +/- PI, or +/_ 180 degrees if you hate radians
    _heading = std::fmod(_heading, 2.0 * _PI); // Constrains the heading so that it must be between -2PI and 2PI, or -360 and 360 degrees (because radians suck)
    if (_heading > _PI) _heading -= 2.0 * _PI; // If we're greater than PI or 180 degrees, subtract 2PI or 360 degrees (added degrees in case you hate radians)
    else if (_heading < -_PI) _heading += 2.0 * _PI; // If we're less than -PI or -180 degrees, add 2PI or 360 degrees (just to be inclusive, I've added degrees in case you hate radians)

    // returns true if the update was successful
    return true;
}

// Position resets are relative. Encoder ticks are not affected by a reset.
void Odometry::resetOdometry()
{
    // TODO
    _x = 0.0;
    _y = 0.0;
    _heading = 0.0;
    return;
}

double Odometry::getX() { return _x; }
double Odometry::getY() { return _y; }
double Odometry::getHeading() { return _heading; }


}