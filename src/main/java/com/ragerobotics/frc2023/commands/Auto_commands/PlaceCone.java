// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.commands.Auto_commands;

import com.ragerobotics.frc2023.Robot;
import com.ragerobotics.frc2023.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PlaceCone extends CommandBase {
  /** Creates a new PlaceCone. */
  public PlaceCone() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.mArm);
    addRequirements(Robot.mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.mIntake.runIntake(-0.4);
    Robot.mArm.positionMaunel(-7440);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.mIntake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
