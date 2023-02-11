

using namespace std;

class RageDrive
{
    public:
        RageDrive();
        double drive();
        double handleDeadband(double val, double deadband);
        double limit(double v, double limit);

    private:
        double wheelDeadband = 0.1;
        double oldWheel = 0.0;
        double quickStopAccumulator;
        double wheel;
        double wheelNonLinearity;
        double negInertia;
        double angularPower;
        double linearPower;
        double leftPwm, rightPwm, overpower;
        double normalTurnSensitivity = .65;
        double quickTurnSensitivity = .69; 
        double negInertiaScalar;
        double overPower;
        double throttle = 0.0 // Robot.GetThrottle();
}