package com.ragerobotics.frc2023.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ragerobotics.frc2023.Constants;
import com.ragerobotics.frc2023.subsystems.wpilib.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Position extends CommandBase {
    public Position() {

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Arm.mot.set(ControlMode.Position, Constants.kArmDoubleStationPosition);
    }
}
