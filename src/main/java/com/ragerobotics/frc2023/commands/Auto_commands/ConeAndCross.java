// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.commands.Auto_commands;

import com.ragerobotics.frc2023.commands.Intake.IntakeIn;
import com.ragerobotics.frc2023.commands.Intake.IntakeOut;
import com.ragerobotics.frc2023.commands.arm.GroundPos;
import com.ragerobotics.frc2023.commands.arm.ScoreHorizontalCone;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeAndCross extends SequentialCommandGroup {
  /** Creates a new ConeAndCross. */
  public ConeAndCross() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PlaceCone().withTimeout(0.7), 
                new RunIntake(0.45).withTimeout(0.4), 
                new GroundPos().withTimeout(0.3),
                new DriveStraight());
  }
}
