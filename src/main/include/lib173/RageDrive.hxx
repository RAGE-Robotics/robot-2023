

using namespace std;

class RageDrive
{
    public:
        RageDrive();

        double handleDeadband(double val, double deadband);

    private:
        double wheelDeadband = 0.1;
        double oldWheel = 0.0;
        double quickStopAccumulator;


}