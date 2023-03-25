// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ragerobotics.frc2023.commands.Auto_commands;

import com.ragerobotics.frc2023.commands.Intake.IntakeOut;
import com.ragerobotics.frc2023.commands.arm.GroundPos;
import com.ragerobotics.frc2023.commands.arm.ScoreCube;
import com.ragerobotics.frc2023.commands.arm.StowedPos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeAndBalance extends SequentialCommandGroup {
  /** Creates a new CubeAndCross. */
  public CubeAndBalance() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // addCommands(new ScoreCube().withTimeout(0.4), new RunIntake(.55).withTimeout(0.4), new StowedPos().withTimeout(0.3), new DriveBalance());
    addCommands(new StowedPos().withTimeout(0.4), new DriveBalance().withTimeout(3.2), new BalancePlatform(0.01, 0, 2, 0.3));
  }
}
