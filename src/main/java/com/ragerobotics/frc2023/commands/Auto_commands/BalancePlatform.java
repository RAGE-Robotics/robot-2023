// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.commands.Auto_commands;

import com.ragerobotics.frc2023.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalancePlatform extends CommandBase {
  /** Creates a new BalancePlatform. */
  private int state;
  private int debounceCount;
  private double speedSlow;
  private double speedFast;
  private double onChargeStationDegree;
  private double levelDegree;
  private double debounceTime;

  public BalancePlatform() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    speedFast = 0.4;
    speedSlow = 0.2;
    onChargeStationDegree = 13.0;
    levelDegree = 0.0;
    debounceTime = 0.2;
  }

  public double autoSpeedBalance() {
    switch(state) {
      case 0:
        if(Robot.mDrive.getTilt() > onChargeStationDegree) {
          debounceCount++;
        }
        // if(debounceCount > secondsToTicks(debounceTime)) {
        //   state = 1;
        //   debounceCount = 0;
        //   return speedSlow;
        // }
      case 1:
        if(Robot.mDrive.getTilt() < levelDegree) {
          debounceCount++;
        }
        // if(debounceCount > secondsToTicks(debounceTime)) {
        //   state = 2;
        //   debounceCount = 0;
        //   return 0;
        // }
        return speedSlow;
      case 2:
        if(Math.abs(Robot.mDrive.getTilt()) <= levelDegree / 2) {
          debounceCount++;
        }
        // if(debounceCount > secondsToTicks(debounceTime)) {
        //   state = 3;
        //   debounceCount = 0;
        //   return 0;
        // }
        if(Robot.mDrive.getTilt() >= levelDegree) {
          return 0.1;
        }
        else if(Robot.mDrive.getTilt() >= levelDegree) {
          return -0.1;
        } 
      
      case 3:
      return 0;
    }
    return 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double aSpeed = autoSpeedBalance();
    // Robot.mDrive.setOpenLoop(aSpeed, aSpeed);
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
