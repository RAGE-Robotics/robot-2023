#include "DifferentialDrivetrain.hxx"

#include <memory>
#include <frc/ADXRS450_Gyro.h>
#include <ctre/Phoenix.h>
#include <cmath>
#include <frc/PS4Controller.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/DriverStation.h>

#include "Constants.hxx"
#include "Controllers.hxx"
#include "lib173/StateEstimator.hxx"

DifferentialDrivetrain::DifferentialDrivetrain() : ahrs{frc::I2C::Port::kMXP}
{
    mLeftPrimaryTalon = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(14);
    mLeftSecondaryTalon = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(13);
    mRightPrimaryTalon = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(1);
    mRightSecondaryTalon = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(5);

    mGearSolenoid = std::make_unique<frc::DoubleSolenoid>(frc::PneumaticsModuleType::REVPH, 7, 5);

    mLeftPrimaryTalon->ConfigFactoryDefault();
    mLeftPrimaryTalon->SetInverted(true);
    mLeftPrimaryTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);

    mLeftSecondaryTalon->ConfigFactoryDefault();
    mLeftSecondaryTalon->SetInverted(true);
    mLeftSecondaryTalon->Follow(*mLeftPrimaryTalon);

    mRightPrimaryTalon->ConfigFactoryDefault();
    mRightPrimaryTalon->SetInverted(false);
    mRightPrimaryTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);

    mRightSecondaryTalon->ConfigFactoryDefault();
    mRightSecondaryTalon->SetInverted(false);
    mRightSecondaryTalon->Follow(*mRightPrimaryTalon);

    shift(false);

    rampRate(0.4);

    setPidGains(Constants::kDrivetrainP, Constants::kDrivetrainI, Constants::kDrivetrainD, Constants::kDrivetrainF);
}

double DifferentialDrivetrain::heading()
{
    return -ahrs.GetAngle() / 360.0 * 2.0 * Constants::kPi;
}

double DifferentialDrivetrain::leftDistance()
{
    return -encoderTicksToMeters(mLeftPrimaryTalon->GetSelectedSensorPosition());
}

double DifferentialDrivetrain::rightDistance()
{
    return -encoderTicksToMeters(mRightPrimaryTalon->GetSelectedSensorPosition());
}

void DifferentialDrivetrain::driveVelocity(double left, double right)
{
    double leftEncoderTicks = metersToEncoderTicks(left) / 10;
    double rightEncoderTicks = metersToEncoderTicks(right) / 10;

    mLeftPrimaryTalon->Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, -(int)std::round(leftEncoderTicks));
    mRightPrimaryTalon->Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, -(int)std::round(rightEncoderTicks));
}

void DifferentialDrivetrain::driveOpenLoop(double left, double right)
{
    mLeftPrimaryTalon->Set(left);
    mRightPrimaryTalon->Set(right);
}

void DifferentialDrivetrain::coast()
{
    mLeftPrimaryTalon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    mLeftSecondaryTalon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    mRightPrimaryTalon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    mRightSecondaryTalon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void DifferentialDrivetrain::brake()
{
    mLeftPrimaryTalon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    mLeftSecondaryTalon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    mRightPrimaryTalon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    mRightSecondaryTalon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void DifferentialDrivetrain::setPidGains(double p, double i, double d, double f)
{
    mLeftPrimaryTalon->Config_kP(0, p);
    mLeftPrimaryTalon->Config_kI(0, i);
    mLeftPrimaryTalon->Config_kD(0, d);
    mLeftPrimaryTalon->Config_kF(0, f);

    mRightPrimaryTalon->Config_kP(0, p);
    mRightPrimaryTalon->Config_kI(0, i);
    mRightPrimaryTalon->Config_kD(0, d);
    mRightPrimaryTalon->Config_kF(0, f);
}

void DifferentialDrivetrain::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> leftdriver = Controllers::instance()->LeftDriver();
    std::shared_ptr<frc::Joystick> rightdriver = Controllers::instance()->RightDriver();
    double l = leftdriver->GetY();
    l = l * fabs(l);
    double r = rightdriver->GetY();
    r = r * fabs(r);
    bool gearUpdated = false;

    if (leftdriver->GetRawButton(1))
    {
        if (highgear)
        {
            shift(false);
            highgear = false;
        }
    }
    if (rightdriver->GetRawButton(1))
    {
        if (!highgear)
        {
            shift(true);
            highgear = true;
        }
    }

    if (mode == 'd')
    {
        if (frc::DriverStation::GetMatchType() != frc::DriverStation::MatchType::kNone)
            brake();
        else
            coast();
        driveOpenLoop(0, 0);
    }
    else if (mode == 't')
    {
        brake();
        driveOpenLoop(l, r);
        if (gearUpdated)
            shift(highgear);
    }

    else if (mode == 'a')
    {
        coast();
    }
}

bool DifferentialDrivetrain::pathFollowing()
{
    mTrajectoryMutex.lock();
    bool result = mTrajectory != nullptr;
    mTrajectoryMutex.unlock();

    return result;
}

void DifferentialDrivetrain::stopPathFollowing()
{
    mTrajectoryMutex.lock();
    mTrajectory = nullptr;
    mTrajectoryMutex.unlock();
}

void DifferentialDrivetrain::followPath(frc::Trajectory path, bool resetPose)
{
    mTrajectoryMutex.lock();
    if (resetPose)
        StateEstimator::instance()->reset(path.InitialPose());
    mTrajectory = std::make_shared<frc::Trajectory>(path);
    mTrajectoryMutex.unlock();
}

void DifferentialDrivetrain::shift(bool isHighGear)
{
    if (isHighGear)
    {
        mGearSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else
    {
        mGearSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    }
}

void DifferentialDrivetrain::rampRate(double rate)
{
    mLeftPrimaryTalon->ConfigOpenloopRamp(0.4);
    mRightPrimaryTalon->ConfigOpenloopRamp(0.4);
}

void DifferentialDrivetrain::resetEncoder()
{
    mLeftPrimaryTalon->SetSelectedSensorPosition(0, 0);
    mRightPrimaryTalon->SetSelectedSensorPosition(0, 0);
}
