// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.commands.Auto_commands;

import com.ragerobotics.frc2023.Robot;
import com.ragerobotics.frc2023.RobotState;
import com.ragerobotics.frc2023.subsystems.Drive;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveBack extends CommandBase {
  /** Creates a new DriveBack. */
  TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory;
  public DriveBack() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory = new TrajectoryIterator<>(new TimedView<>(Robot.mTrajectoryGenerator.getTrajectorySet().driveReverse));
    RobotState.getInstance().reset(Timer.getFPGATimestamp(), trajectory.getState().state().getPose());
    Robot.mDrive.setTrajectory(trajectory);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Drive.getInstance().isDoneWithTrajectory();
  }
}
