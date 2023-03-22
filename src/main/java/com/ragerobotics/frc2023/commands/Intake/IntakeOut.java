package com.ragerobotics.frc2023.commands.Intake;

import com.ragerobotics.frc2023.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeOut extends CommandBase {
    public IntakeOut() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.mIntake);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {}
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
         Robot.mIntake.runIntake(0.6);
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        Robot.mIntake.runIntake(0);
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
}
