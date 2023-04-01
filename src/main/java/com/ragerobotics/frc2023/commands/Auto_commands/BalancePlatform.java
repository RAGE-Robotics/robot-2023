// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.commands.Auto_commands;

import com.ragerobotics.frc2023.Robot;
import com.ragerobotics.frc2023.subsystems.Drive;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalancePlatform extends CommandBase {
  /** Creates a new BalancePlatform. */
  private double error;
  private double mKp;
  private double aSpeed;
  private double pidSpeed;
  private double aDegree;
  private double tolerance;

  public BalancePlatform(double kP, double setDegrees, double tolerate, double mSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mKp = kP;
    this.aDegree = setDegrees;
    this.tolerance = tolerate;
    this.aSpeed = mSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = Math.abs(aDegree - Robot.mDrive.NAVXpitch());

    if (mKp * error >= aSpeed) {
      pidSpeed = aSpeed;
    } else {
      pidSpeed = mKp * error;
    }

    if (Robot.mDrive.NAVXpitch() < aDegree) {
      Drive.getInstance().setOpenLoop(new DriveSignal(-pidSpeed, -pidSpeed));
    } else {
      Drive.getInstance().setOpenLoop(new DriveSignal(pidSpeed, pidSpeed));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.mDrive.setOpenLoop(new DriveSignal(0, 0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return ((Math.abs(error) <= tolerance));
    return false;
  }
}
