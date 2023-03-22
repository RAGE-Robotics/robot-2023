package com.ragerobotics.frc2023.subsystems.wpilib;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonSRX mIntakeLeftMotor = new TalonSRX(7);
    private final TalonSRX mIntakeRightMotor = new TalonSRX(11);

    public Intake() {
        mIntakeLeftMotor.setNeutralMode(NeutralMode.Coast);
        mIntakeRightMotor.setNeutralMode(NeutralMode.Coast);
        mIntakeLeftMotor.configFactoryDefault();
        mIntakeRightMotor.configFactoryDefault();
        mIntakeRightMotor.follow(mIntakeLeftMotor, FollowerType.PercentOutput);
        mIntakeRightMotor.setInverted(false);
        mIntakeLeftMotor.setInverted(true);
    }

    @Override
    public void periodic() {

    }
}
