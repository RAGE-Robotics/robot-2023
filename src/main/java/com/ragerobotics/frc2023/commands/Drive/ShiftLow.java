package com.ragerobotics.frc2023.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShiftLow extends CommandBase {
    public ShiftLow() {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(Robot.<--insertSubsystemHere-->);
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
