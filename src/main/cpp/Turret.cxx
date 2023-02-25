#include "Turret.hxx"

#include "Constants.hxx"
#include "Controllers.hxx"
#include "lib173/StateEstimator.hxx"
#include <frc/smartdashboard/SmartDashboard.h>

Turret::Turret()
{
    mTurret = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(7);
    // mTurret->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);

    mTurret->ConfigMotionCruiseVelocity(4332);
    mTurret->ConfigMotionAcceleration(2332);

    mTurret->Config_kP(0, 10);
    mTurret->Config_kI(0, 0);
    mTurret->Config_kD(0, 0);
    mTurret->Config_kF(0, 0);
}

Turret::~Turret()
{
}

bool Turret::homingSwitchActive()
{
    return true;
}

// double Turret::encoderPosition()
// // {
// //     turretPosition = mTurret->GetSelectedSensorPosition() / Constants::kTurretEncoderUnitsPerRotation;
// //     frc::SmartDashboard::PutNumber("Turret", turretPosition);
// //     return turretPosition;
// }

void Turret::manualMode(double percentPower)
{
    mTurret->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, percentPower * 0.3);
}

// void Turret::magicalTwist()
// {
//     manual = false;
//     mTurret->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::MotionMagic, 200);
// }
void Turret::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> opr = Controllers::instance()->RightOperator();

    // encoderPosition();
    // manual = true;

    double rotate = opr->GetX();

    if (mode == 't')
    {
        // if (manual)
        // {
            manualMode(rotate);
        // }
    }
}