package com.ragerobotics.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends RageSubsystem {
    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }

    private final TalonSRX mIntakeLeftMotor = new TalonSRX(7);
    private final TalonSRX mIntakeRightMotor = new TalonSRX(11);

    private Intake() {
        mIntakeLeftMotor.configFactoryDefault();
        mIntakeRightMotor.configFactoryDefault();

        mIntakeLeftMotor.setNeutralMode(NeutralMode.Coast);
        mIntakeRightMotor.setNeutralMode(NeutralMode.Coast);

        mIntakeRightMotor.follow(mIntakeLeftMotor, FollowerType.PercentOutput);
        mIntakeRightMotor.setInverted(false);
        mIntakeLeftMotor.setInverted(true);
    }

    @Override
    public void update(double timestamp, char mode) {
        if (mode == 'a' || mode == 't') {
            mIntakeLeftMotor.set(ControlMode.PercentOutput, 0.5);
        }
    }
}
