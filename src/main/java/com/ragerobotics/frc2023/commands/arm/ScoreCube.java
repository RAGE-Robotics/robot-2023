package com.ragerobotics.frc2023.commands.arm;

import com.ragerobotics.frc2023.Constants;
import com.ragerobotics.frc2023.Robot;
import com.ragerobotics.frc2023.subsystems.wpilib.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreCube extends CommandBase {

    public ScoreCube(){
        addRequirements(Robot.mArm);
    }
    
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      Robot.mArm.positionMaunel(Constants.kArmCubePlaceHeightPosition);
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
