// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.commands.Auto_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ragerobotics.frc2023.Robot;

public class DriveStraight extends CommandBase {
  /** Creates a new DriveStraight. */
  public DriveStraight() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory = new TrajectoryIterator<>(new TimedView<>(Robot.mTrajectoryGenerator.getTrajectorySet().testTrajectory));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
