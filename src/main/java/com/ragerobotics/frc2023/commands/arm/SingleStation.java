// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.commands.arm;

import com.ragerobotics.frc2023.Constants;
import com.ragerobotics.frc2023.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SingleStation extends CommandBase {
  /** Creates a new SingleStation. */
  public SingleStation() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.mArm.positionMaunel(Constants.kArmSingleStationPosition);
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
