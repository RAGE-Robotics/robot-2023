package com.ragerobotics.frc2023.subsystems.wpilib;

import com.ragerobotics.frc2023.Constants;
import com.ragerobotics.frc2023.subsystems.Drive;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class DriveTrain extends SubsystemBase {
  public DriveTrain() {
  }

  // public void coast() {
  //   Drive.getInstance().setBrakeMode(false);
  // }

  // public void brake() {
  //   Drive.getInstance().setBrakeMode(true);
  // }

  public void drive(double leftPercent, double rightPercent) {
    Drive.getInstance().setOpenLoop(new DriveSignal(leftPercent, rightPercent));
    
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
