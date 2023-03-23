package com.ragerobotics.frc2023.commands.arm;

import com.ragerobotics.frc2023.Constants;
import com.ragerobotics.frc2023.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreHorizontalCone extends CommandBase {

    public ScoreHorizontalCone(){
        addRequirements(Robot.mArm);
    }
    
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      Robot.mArm.positionMaunel(Constants.kArmHorizontalConePlacePosition);
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
