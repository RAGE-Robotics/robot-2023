#pragma once

#include <memory>
#include <frc/ADXRS450_Gyro.h>
#include <ctre/Phoenix.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/DoubleSolenoid.h>

#include "lib173/Drivetrain.hxx"
#include "System.hxx"
#include "Constants.hxx"
#include "AHRS.h"

class DifferentialDrivetrain : public Drivetrain, public System
{
private:

    bool highgear;
    static double encoderTicksToMeters(int encoderTicks)
    {
        double rotations = encoderTicks;
        rotations /= Constants::kDrivetrainEncoderUnitsPerRotation;
        rotations *= Constants::kDrivetrainGearRatio;
        double radians = rotations * 2 * Constants::kPi;

        double meters = radians * Constants::kDrivetrainWheelRadius;
        return meters;
    }

    static double metersToEncoderTicks(double meters)
    {
        double radians = meters / Constants::kDrivetrainWheelRadius;
        double rotations = radians / (2 * Constants::kPi) / Constants::kDrivetrainGearRatio;

        double encoderTicks = rotations * Constants::kDrivetrainEncoderUnitsPerRotation;
        return encoderTicks;
    }

    std::unique_ptr<frc::ADXRS450_Gyro> mGyro;

    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonFX> mLeftPrimaryTalon;
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonFX> mLeftSecondaryTalon;
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonFX> mRightPrimaryTalon;
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonFX> mRightSecondaryTalon;

    std::unique_ptr<frc::DoubleSolenoid> mGearSolenoid;
    
    

public:
    static std::shared_ptr<DifferentialDrivetrain> instance()
    {
        static std::shared_ptr<DifferentialDrivetrain> drivetrain = std::make_shared<DifferentialDrivetrain>();
        return drivetrain;
    }

    DifferentialDrivetrain();

    double heading() override;
    double leftDistance() override;
    double rightDistance() override;
    void driveVelocity(double left, double right) override;
    void driveOpenLoop(double left, double right);

    void coast();
    void brake();

    void setPidGains(double p, double i, double d, double f) override;

    void updateSystem(double timestamp, char mode) override;

    bool pathFollowing();
    void stopPathFollowing();
    void followPath(frc::Trajectory path, bool resetPose = true);
    void resetEncoder();

    void shift(bool isHighGear);

    void rampRate(double rate);

    AHRS ahrs;
};
