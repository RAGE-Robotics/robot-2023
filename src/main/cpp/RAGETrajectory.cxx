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
        40_ft,
        20_ft,
        frc::Rotation2d(0_deg)};
    frc::TrajectoryConfig Config{
        18_fps,
        5_fps_sq,
    };

    std::vector<frc::Translation2d> interiorWayPts{
        frc::Translation2d{2_ft, 0_ft},
        frc::Translation2d{5_ft, 2_ft},
        frc::Translation2d{10_ft, 4_ft},
        frc::Translation2d{15_ft, 6_ft},
        frc::Translation2d{25_ft, 10_ft},
        frc::Translation2d{30_ft, 15_ft},
        frc::Translation2d{35_ft, 20_ft}};

    auto Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        firstPoint,
        interiorWayPts,
        secondPoint,
        Config);

    return Trajectory;
}
