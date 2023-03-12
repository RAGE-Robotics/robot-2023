#pragma once

#include <ctre/Phoenix.h>
#include <memory>
#include <frc/trajectory/Trajectory.h>
#include <Claw.hxx>

class RAGETrajectory
{
private:
    int autoStages = 0;
    std::shared_ptr<Claw> claw = Claw::instance();
public:
    RAGETrajectory();
    static std::shared_ptr<RAGETrajectory> instance()
    {
        static std::shared_ptr<RAGETrajectory> trajectory = std::make_shared<RAGETrajectory>();
        return trajectory;
    }
<<<<<<< HEAD
    frc::Trajectory RedBalance();
    frc::Trajectory BlueBalance();
    frc::Trajectory DriveStraight();
=======
    frc::Trajectory GeneratePoints();

    void runAutonomous();

>>>>>>> d0cfa282c1ab5b6dfcb4b9439ec8328a27df84fd
    void SetConfig();
};
