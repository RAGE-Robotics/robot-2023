#include "RAGETrajectory.hxx"
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <Claw.hxx>

#include "Constants.hxx"

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
        -7.0_ft,
        // 6_ft,
        0_ft,
        frc::Rotation2d(0_deg)};
    frc::TrajectoryConfig Config{
        6_fps,
        3_fps_sq
    };
    Config.SetReversed(true);

    std::vector<frc::Translation2d> interiorWayPts{
        frc::Translation2d{-2_ft, 0_ft},
        frc::Translation2d{-5_ft, 0_ft},
        // frc::Translation2d{9.5_ft, 0_ft},
        // frc::Translation2d{14_ft, 0_ft}
        };

    auto Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        firstPoint,
        interiorWayPts,
        secondPoint,
        Config);

    return Trajectory;
}

// void RAGETrajectory::runAutonomous() {
//     switch (autoStages) {
//         case 0:
//             claw->intakeRollersIn(1);
//             autoStages++;
//         case 1:
//             claw->intakeRollersIn(0);
//             // Claw::moveWrist(true)
//             // GeneratePoints();
//             autoStages++;
//     }
// }
