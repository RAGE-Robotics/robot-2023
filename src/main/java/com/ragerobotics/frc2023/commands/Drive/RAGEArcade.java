package com.ragerobotics.frc2023.commands.Drive;

import com.ragerobotics.frc2023.Constants;
import com.ragerobotics.frc2023.Controllers;
import com.ragerobotics.frc2023.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RAGEArcade extends CommandBase {

    public RAGEArcade() {
        addRequirements(Robot.driveTrain);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (DriverStation.isAutonomous())
            return;

        double throttle = -Controllers.getLeftDriverY();
        double steer = Controllers.getRightDriverX() * 0.52;

        if (Math.abs(throttle) < Constants.kArcadeDriveDeadband) {
            throttle = 0;
        }
        if (Math.abs(steer) < Constants.kArcadeDriveDeadband) {
            steer = 0;
        }

        boolean throttleNegative = throttle < 0;
        boolean steerNegative = steer < 0;

        throttle *= throttle;
        steer *= steer;

        if (throttleNegative) {
            throttle *= -1;
        }
        if (steerNegative) {
            steer *= -1;
        }

        Robot.driveTrain.drive(throttle + steer, throttle - steer);
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
