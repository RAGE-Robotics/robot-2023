#include "Claw.hxx"
#include "Controllers.hxx"

Claw::Claw()
{
    mClawIntake = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(11);
    mWrist = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(8);

    mWrist->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

double Claw::encoderCounts()
{
    return mWrist->GetSelectedSensorPosition();
}

void Claw::moveWrist(double wristpower)
{
    mWrist->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, wristpower * 0.5);
}

void Claw::intakeRollersIn(double intakepower)
{
    mClawIntake->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, intakepower * 0.5);
}
void Claw::intakeRollersOut(double intakepower)
{
    mClawIntake->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, intakepower * -0.5);
}
void Claw::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> opl = Controllers::instance()->LeftOperator();
    std::shared_ptr<frc::Joystick> opr = Controllers::instance()->RightOperator();

    bool cone = opl->GetRawButton(1);
    bool cube = opr->GetRawButton(1);
    double wrist = 1;
    if (mode == 't')
    {
        if (opr->GetRawButton(3))
        {
            moveWrist(wrist);
        }
        else if (opr->GetRawButton(2))
        {
            moveWrist(-wrist);
        }
        else
        {
            moveWrist(0);
        }

        if (cone)
        {
            intakeRollersIn(1);
        }
        else if (cube)
        {
            intakeRollersOut(1);
        }
        else
        {
            intakeRollersIn(0);
            intakeRollersOut(0);
        }
    }
}
