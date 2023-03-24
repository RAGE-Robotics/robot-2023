// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.commands.Auto_commands;

import com.ragerobotics.frc2023.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntake extends CommandBase {
  /** Creates a new RunIntake. */
  private double intakePower;

  public RunIntake(double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.mIntake);
    this.intakePower = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Robot.mIntake.runIntake(intakePower);
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
