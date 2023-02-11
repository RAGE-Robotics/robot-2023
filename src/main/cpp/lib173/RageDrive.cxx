#include "RageDrive.hxx"
#include "math.h"

using namespace std;


RageDrive::RageDrive() {
}

double RageDrive::drive() {
    double negInertia = wheel - oldWheel;
    oldWheel = wheel;

    wheelNonLinearity = 0.5;

    wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) /
        sin(M_PI / 2.0 * wheelNonLinearity);
    wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) /
        sin(M_PI / 2.0 * wheelNonLinearity);
    wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) /
        sin(M_PI / 2.0 * wheelNonLinearity);
    
    double negInertiaAccumulator = 0;
    
    if(wheel * negInertia > 0) {
        negInertiaScalar = 2.5;
    }
    else {
        if(abs(wheel) > 0.65) {
            negInertiaScalar = 5.0;
        }
        else {
            negInertiaScalar = 3.0;
        }
    }

    double negInertiaPower = negInertia * negInertiaScalar;
    negInertiaAccumulator = negInertiaPower;

    wheel = wheel + negInertiaAccumulator;
    if(negInertiaAccumulator > 1) {
        negInertiaAccumulator -= 1;
    }
    else if (negInertiaAccumulator < -1) {
        negInertiaAccumulator += 1;
    }
    else {
        negInertiaAccumulator = 0;
    }

    // linearPower = throttle;

    // Quickturn!
    if(abs(linearPower) < 0.2) {
        double alpha = 0.1;
        quickStopAccumulator = (1-alpha) * quickStopAccumulator + alpha *
            limit(wheel, 1.0) * 5;
        overpower = 1.0;
        normalTurnSensitivity = 1.0;

        angularPower = quickTurnSensitivity * wheel;
    }

    else {
        overPower = 0.0;
        angularPower = abs(throttle) * wheel * normalTurnSensitivity - quickStopAccumulator;
        if(quickStopAccumulator > 1) {
            quickStopAccumulator -= 1;
        }
        else if (quickStopAccumulator < -1) {
            quickStopAccumulator += 1;
        }
        else {
            quickStopAccumulator = 0;
        }

        rightPwn = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;

        if(leftPwm > 1.0) {
            rightPwm -= overpower * (leftPwm - 1.0);
            leftPwm = 1.0;
        }

        else if(rightPwm > 1.0) {
            rightPwm -= overpower * (rightPwm - 1.0);
            leftPwm = 1.0;
        }

        else if(leftPwm < -1.0) {
            leftPwm _+= overpower * (rightPwm - 1.0);
            rightPwm = 1.0;
        }

        else if(rightPwm < -1.0) {
            leftPwm -= overpower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }
    }

    // call something from robot.cpp to drive the robot;
    return (leftPwm, rightPwm)
}


double RageDrive::limit(double v, double limit) {

}

double RageDrive::handleDeadband(double val, double deadband) {
       return (abs(val) > abs(deadband)) ? val : 0.0;
}