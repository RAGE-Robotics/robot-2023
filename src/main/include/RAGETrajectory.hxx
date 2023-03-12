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
    frc::Trajectory GeneratePoints();

    void runAutonomous();

    void SetConfig();
};
