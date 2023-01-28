#include "Arm.hxx"

using namespace std;
Arm::Arm(){
   mArmRaiser = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(0);
   mArmExtender = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(1);
}
double Arm::encoderCounts(){
    return mArmRaiser->GetSelectedSensorPosition();
}

void Arm::raiseArm(double percent){
    mArmRaiser->Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput, percent);
}