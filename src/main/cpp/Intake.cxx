#include "Intake.hxx"

using namespace std;
Claw::Claw()
{
    mClawIntake = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(9);
    mWrist = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(8);
}
double Claw::encoderCounts()
{
    return mWrist->GetSelectedSensorPosition();
}

void Claw::moveWrist(double wristpower){
    mWrist->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, wristpower * 0.5);
}

void Claw::intakeRollersIn(double intakepower){
    mClawIntake->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, intakepower * 0.5);

}
void Claw::intakeRollersOut(double intakepower){
    mClawIntake->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, intakepower * -0.5);

}

