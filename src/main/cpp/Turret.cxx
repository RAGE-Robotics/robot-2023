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

    mTurret->Config_kP(0, .034);
    mTurret->Config_kI(0, 0);
    mTurret->Config_kD(0, .055);
    mTurret->Config_kF(0, 0);

    mTurret->SetSelectedSensorPosition(0, 0);
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
    adjustment = 500 * percentPower;
}

double Turret::encoderPosition()
{
    turretEncoderPosition = mTurret->GetSelectedSensorPosition();
    return turretEncoderPosition;
}

void Turret::setTurretAngle(double radians)
{
    sensorPos = radians * Constants::kTurretEncoderTicksPerRadian;

    mTurret->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Position, sensorPos);
}

void Turret::brake()
{
    mTurret->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void Turret::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> opr = Controllers::instance()->RightOperator();

    double rotate = opr->GetX();
    bool r = opr->GetRawButton(7);
    bool zero = opr->GetRawButton(6);
    bool l = opr->GetRawButton(11);

    if (mode == 't')
    {
        manualMode(rotate);

        if (r)
        {
            setTurretAngle((Constants::kPi / 2));
        }
        else if (zero)
        {
            setTurretAngle(0);
        }
        else if (l)
        {
            setTurretAngle(-(Constants::kPi / 2));
        }
    }

    if (mode == 't' || mode == 'a')
    {
        setTurretAngle((sensorPos + adjustment) / Constants::kTurretEncoderTicksPerRadian);
        brake();
    }
}
