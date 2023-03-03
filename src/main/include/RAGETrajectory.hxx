#pragma once

#include <ctre/Phoenix.h>
#include <memory>
#include <frc/trajectory/Trajectory.h>

class RAGETrajectory
{
private:

public:
    RAGETrajectory();
    static std::shared_ptr<RAGETrajectory> instance()
    {
        static std::shared_ptr<RAGETrajectory> trajectory = std::make_shared<RAGETrajectory>();
        return trajectory;
    }
    frc::Trajectory GeneratePoints();
    void SetConfig();
};
