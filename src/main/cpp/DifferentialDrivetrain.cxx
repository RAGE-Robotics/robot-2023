#include "DifferentialDrivetrain.hxx"

#include <memory>
#include <frc/ADXRS450_Gyro.h>
#include <ctre/Phoenix.h>
#include <cmath>
#include <frc/PS4Controller.h>
#include <frc/trajectory/Trajectory.h>

#include "Constants.hxx"
#include "Controllers.hxx"
#include "lib173/StateEstimator.hxx"

DifferentialDrivetrain::DifferentialDrivetrain()
{
    // mGyro = std::make_unique<frc::ADXRS450_Gyro>(frc::SPI::Port::kOnboardCS0);

    mLeftPrimaryTalon = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(2);
    mLeftSecondaryTalon = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(1);
    mRightPrimaryTalon = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(3);
    mRightSecondaryTalon = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(4);

    mLeftPrimaryTalon->ConfigFactoryDefault();
    mLeftPrimaryTalon->SetInverted(true);
    mLeftPrimaryTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);

    mLeftSecondaryTalon->ConfigFactoryDefault();
    mLeftSecondaryTalon->SetInverted(true);
    mLeftSecondaryTalon->Follow(*mLeftPrimaryTalon);

    mRightPrimaryTalon->ConfigFactoryDefault();
    mRightPrimaryTalon->SetInverted(true);
    mRightPrimaryTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);
    mRightPrimaryTalon->SetSensorPhase(true);

    mRightSecondaryTalon->ConfigFactoryDefault();
    mRightSecondaryTalon->SetInverted(true);
    mRightSecondaryTalon->Follow(*mRightPrimaryTalon);

    coast();
    setPidGains(Constants::kDrivetrainP, Constants::kDrivetrainI, Constants::kDrivetrainD, Constants::kDrivetrainF);
}

double DifferentialDrivetrain::heading()
{
    // return mGyro->GetAngle();
    return 0;
}

double DifferentialDrivetrain::leftDistance()
{
    return encoderTicksToMeters(mLeftPrimaryTalon->GetSelectedSensorPosition());
}

double DifferentialDrivetrain::rightDistance()
{
    return encoderTicksToMeters(mRightPrimaryTalon->GetSelectedSensorPosition());
}

void DifferentialDrivetrain::driveVelocity(double left, double right)
{
    double leftEncoderTicks = metersToEncoderTicks(left) / 10;
    double rightEncoderTicks = metersToEncoderTicks(right) / 10;

    mLeftPrimaryTalon->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Velocity, (int)std::round(leftEncoderTicks));
    mRightPrimaryTalon->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Velocity, (int)std::round(rightEncoderTicks));
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
    std::shared_ptr<frc::PS4Controller> driver = Controllers::instance()->driver();
    double x = driver->GetLeftX();
    x = x * fabs(x);
    double y = driver->GetLeftY();
    y = y * fabs(y);
    y *= -1.0;

    if (mode == 'd')
        driveOpenLoop(0, 0);
    else if (mode == 't')
        driveOpenLoop(y + x, y - x);
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
