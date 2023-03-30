// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.commands.Auto_commands;

import com.ragerobotics.frc2023.commands.Intake.IntakeOut;
import com.ragerobotics.frc2023.commands.arm.GroundPos;
import com.ragerobotics.frc2023.commands.arm.ScoreCube;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeAndCross extends SequentialCommandGroup {
  /** Creates a new CubeAndCross. */
  public CubeAndCross() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new ScoreCube().withTimeout(0.6), 
                new RunIntake(.6).withTimeout(0.4), 
                new GroundPos().withTimeout(0.3), 
                new PickupCube().withTimeout(3.5)
                );
  }
}
