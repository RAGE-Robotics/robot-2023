#include "RAGETrajectory.hxx"
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

RAGETrajectory::RAGETrajectory()
{
}

frc::Trajectory RAGETrajectory::GeneratePoints()
{
    const frc::Pose2d firstPoint{
        0_ft,
        0_ft,
        frc::Rotation2d(0_deg)};
    const frc::Pose2d secondPoint{
        // 16.5_ft,
        5_ft,
        0_ft,
        frc::Rotation2d(0_deg)};
    frc::TrajectoryConfig Config{
        7_fps,
        2_fps_sq,
    };

    std::vector<frc::Translation2d> interiorWayPts{
        frc::Translation2d{2_ft, 0_ft},
        frc::Translation2d{4_ft, 0_ft},
        // frc::Translation2d{10_ft, 0_ft},
        // frc::Translation2d{14_ft, 0_ft}
        };

    auto Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        firstPoint,
        interiorWayPts,
        secondPoint,
        Config);

    return Trajectory;
}
