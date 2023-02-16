#pragma once

#include "Robot.hxx"
#include <ctre/Phoenix.h>
#include <memory.h>
#include <frc/trajectory/Trajectory.h>

class RAGETrajectory
{
private:

public:
    RAGETrajectory();
    void GeneratePoints();
};
