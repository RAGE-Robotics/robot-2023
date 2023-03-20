package com.ragerobotics.frc2023.subsystems.wpilib;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonSRX mIntakeLeftMotor = new TalonSRX(8);
    private final TalonSRX mIntakeRightMotor = new TalonSRX(11);

    public Intake() {
        mIntakeRightMotor.follow(mIntakeLeftMotor, FollowerType.PercentOutput);
        mIntakeRightMotor.setInverted(true);
    }

    @Override
    public void periodic() {

    }
}
