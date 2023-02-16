#include "RAGETrajectory.hxx"
#include <frc/trajectory/Trajectory.h>

RAGETrajectory::RAGETrajectory()
{
}

void RAGETrajectory::GeneratePoints()
{
    const frc::Pose2d firstPoint
    {
        5_ft,
        0_ft, 
        frc::Rotation2d(0_deg)
    };
    const frc::Pose2d secondPoint
    {
        5_ft,
        0_ft,
        frc::Rotation2d(0_deg)
    };
    frc::TrajectoryConfig Config
    {
        4_fps, 
        2_fps_sq,
    };
    auto Trajectory = frc::TrajectoryGenerator::GenerateTrajectory()
    {
        firstPoint, secondPoint, Config
    }
}

