package com.ragerobotics.frc2023.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConeIn extends CommandBase {
    public ConeIn() {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(Robot.claw);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {}
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        // Robot.claw.RunIntake(-1);
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
