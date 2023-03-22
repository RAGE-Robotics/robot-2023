package com.ragerobotics.frc2023.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ragerobotics.frc2023.Constants;
import com.ragerobotics.frc2023.Controllers;
import com.ragerobotics.frc2023.Robot;
import com.team254.lib.util.DriveSignal;

public class RAGEDrive extends CommandBase {
  private double oldWheel = 0.0;
  private double quickStopAccumulator;
  private double wheelDeadband = 0.1;

  public RAGEDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double wheelNonLinearity;
    double wheel;

    wheel = handleDeadband(Controllers.GetTurn(), wheelDeadband);

    double throttle = Controllers.GetThrottle();

    double negInertia = wheel - oldWheel;
    oldWheel = wheel;

    wheelNonLinearity = 0.5;
    // Apply a sin function that's scaled to make it feel better.
    wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) /
        Math.sin(Math.PI / 2.0 * wheelNonLinearity);
    wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) /
        Math.sin(Math.PI / 2.0 * wheelNonLinearity);
    wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) /
        Math.sin(Math.PI / 2.0 * wheelNonLinearity);

    double leftPwm, rightPwm, overPower;
    double sensitivity = Constants.kNormalTurnSensitivity;

    double angularPower;
    double linearPower;

    // Negative inertia!
    double negInertiaAccumulator = 0.0;
    double negInertiaScalar;
    if (wheel * negInertia > 0) {
      negInertiaScalar = 2.5;
    } else {
      if (Math.abs(wheel) > 0.65) {
        negInertiaScalar = 5.0;
      } else {
        negInertiaScalar = 3.0;
      }
    }
    sensitivity = Constants.kNormalTurnSensitivity;

    if (Math.abs(throttle) > 0.1) {
    }

    double negInertiaPower = negInertia * negInertiaScalar;
    negInertiaAccumulator += negInertiaPower;

    wheel = wheel + negInertiaAccumulator;
    if (negInertiaAccumulator > 1) {
      negInertiaAccumulator -= 1;
    } else if (negInertiaAccumulator < -1) {
      negInertiaAccumulator += 1;
    } else {
      negInertiaAccumulator = 0;
    }
    linearPower = throttle;

    // Quickturn!
    // if (isQuickTurn) {
    if (Math.abs(linearPower) < 0.2) {
      double alpha = 0.1;
      quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha *
          limit(wheel, 1.0) * 5;
      // }
      overPower = 1.0;
      sensitivity = 1.0;

      angularPower = Constants.kQuickturnSensitivity * wheel;
    } else {
      overPower = 0.0;
      angularPower = Math.abs(throttle) * wheel * sensitivity - quickStopAccumulator;
      if (quickStopAccumulator > 1) {
        quickStopAccumulator -= 1;
      } else if (quickStopAccumulator < -1) {
        quickStopAccumulator += 1;
      } else {
        quickStopAccumulator = 0.0;
      }
    }

    rightPwm = leftPwm = linearPower;
    leftPwm += angularPower;
    rightPwm -= angularPower;

    if (leftPwm > 1.0) {
      rightPwm -= overPower * (leftPwm - 1.0);
      leftPwm = 1.0;
    } else if (rightPwm > 1.0) {
      leftPwm -= overPower * (rightPwm - 1.0);
      rightPwm = 1.0;
    } else if (leftPwm < -1.0) {
      rightPwm += overPower * (-1.0 - leftPwm);
      leftPwm = -1.0;
    } else if (rightPwm < -1.0) {
      leftPwm += overPower * (-1.0 - rightPwm);
      rightPwm = -1.0;
    }

    // Robot.mDrive.setOpenLoop(new DriveSignal(leftPwm, rightPwm)); // MAKE SURE TO UNCOMMENT DIS
  }

  public double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public static double limit(double v, double limit) {
    return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
