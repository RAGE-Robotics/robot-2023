package com.ragerobotics.frc2023.subsystems.wpilib;

import com.ragerobotics.frc2023.Constants;
import com.ragerobotics.frc2023.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class DriveTrain extends SubsystemBase {
  final TalonFX leftLeader = new TalonFX(14);
  final TalonFX leftFollower = new TalonFX(13);
  final TalonFX rightLeader = new TalonFX(1);
  final TalonFX rightFollower = new TalonFX(5);

  public DriveTrain() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
  }

  public void coast() {
    leftLeader.setNeutralMode(NeutralMode.Coast);
    rightLeader.setNeutralMode(NeutralMode.Coast);
  }

  public void brake() {
    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
  }

  public void drive(double leftPercent, double rightPercent) {
    leftLeader.set(ControlMode.PercentOutput, leftPercent);
    rightLeader.set(ControlMode.PercentOutput, rightPercent);
  }

  public void setRampRate(double rampRate) {
    leftLeader.configOpenloopRamp(Constants.kDriveRampRate);
    rightLeader.configOpenloopRamp(Constants.kDriveRampRate);
  }


  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
