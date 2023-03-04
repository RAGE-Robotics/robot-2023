#include "Turret.hxx"

#include "Constants.hxx"
#include "Controllers.hxx"
#include "lib173/StateEstimator.hxx"
#include <frc/smartdashboard/SmartDashboard.h>

Turret::Turret()
{
    mTurret = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(7);
    mTurret->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);

    mTurret->SetSensorPhase(true);

    mTurret->Config_kP(0, .03);
    mTurret->Config_kI(0, .04);
    mTurret->Config_kD(0, .05);
    mTurret->Config_kF(0, 0);
}

Turret::~Turret()
{
}

bool Turret::homingSwitchActive()
{
    // if (turretLimitSwitch.Get())
    // {
    //     mTurret->SetSelectedSensorPosition(0);
    // }
    homingSwitch = mTurret->GetSelectedSensorPosition();
    return homingSwitch;
}


void Turret::manualMode(double percentPower)
{
    mTurret->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, percentPower * 0.5);
}

double Turret::encoderPosition()
{
    turretEncoderPosition = mTurret->GetSelectedSensorPosition();
    return turretEncoderPosition;
}

void Turret::setTurretAngle(double radians)
{
    sensorPos = radians * Constants::kTurretEncoderTicksPerRadian;
    // error = abs(sensorPos - mTurret->GetSelectedSensorPosition());

    std::cout << "setting turret position" << std::endl;

    mTurret->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Position, sensorPos);
}

void Turret::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> opr = Controllers::instance()->RightOperator();

    // encoderPosition();
    // manual = true;

    double rotate = opr->GetX();
    bool r = opr->GetRawButton(7);

    if (mode == 't')
    {
        // if (manual)
        // {
        manualMode(rotate);

        if (r)
        {
            setTurretAngle((Constants::kPi / 2));
        }

        // }
    }
}