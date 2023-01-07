#include "lib173/StateEstimator.hxx"

#include <memory>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include "lib173/Drivetrain.hxx"
#include "Constants.hxx"

StateEstimator::StateEstimator()
{
    frc::DifferentialDriveKinematics kinematics{units::meter_t{Constants::kDrivetrainWidth}};
    mPoseEstimator = std::make_unique<frc::DifferentialDrivePoseEstimator>(kinematics, frc::Rotation2d{}, units::meter_t{0}, units::meter_t{0}, frc::Pose2d{});
    mDrivetrain = nullptr;
}

void StateEstimator::setDrivetrain(std::shared_ptr<Drivetrain> drivetrain)
{
    mPoseEstimatorMutex.lock();
    mDrivetrain = drivetrain;
    mPoseEstimatorMutex.unlock();
}

void StateEstimator::reset(frc::Pose2d pose)
{
    mPoseEstimatorMutex.lock();
    mPoseEstimator->ResetPosition(mDrivetrain ? frc::Rotation2d{units::radian_t{mDrivetrain->heading()}} : frc::Rotation2d{}, units::meter_t{mDrivetrain ? mDrivetrain->leftDistance() : 0}, units::meter_t{mDrivetrain ? mDrivetrain->rightDistance() : 0}, pose);
    mPoseEstimatorMutex.unlock();
}

void StateEstimator::update(double timestamp)
{
    frc::Rotation2d heading;
    units::meter_t leftDistance{0}, rightDistance{0};

    mPoseEstimatorMutex.lock();

    if (mDrivetrain)
    {
        heading = frc::Rotation2d{units::radian_t{mDrivetrain->heading()}};
        leftDistance = units::meter_t{mDrivetrain->leftDistance()};
        rightDistance = units::meter_t{mDrivetrain->rightDistance()};
    }

    mPoseEstimator->Update(heading, leftDistance, rightDistance);
    mPoseEstimatorMutex.unlock();
}

void StateEstimator::updateVision(frc::Pose2d pose, double timestamp)
{
    mPoseEstimatorMutex.lock();
    mPoseEstimator->AddVisionMeasurement(pose, units::second_t{timestamp});
    mPoseEstimatorMutex.unlock();
}

frc::Pose2d StateEstimator::pose()
{
    mPoseEstimatorMutex.lock();
    frc::Pose2d pose = mPoseEstimator->GetEstimatedPosition();
    mPoseEstimatorMutex.unlock();

    return pose;
}
