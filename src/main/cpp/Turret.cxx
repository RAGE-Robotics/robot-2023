#include "Turret.hxx"

#include "Constants.hxx"
#include "Controllers.hxx"
#include "lib173/StateEstimator.hxx"


Turret::Turret()
{
    mTurret = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(2);
}

Turret::~Turret(){

}

bool Turret::homingSwitchActive()
{
    return true;
}

double Turret::encoderPosition()
{
    return mTurret->GetSelectedSensorPosition();
}

void Turret::rotateTurret(double percentPower)
{
    mTurret->Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput, percentPower * 20);
}