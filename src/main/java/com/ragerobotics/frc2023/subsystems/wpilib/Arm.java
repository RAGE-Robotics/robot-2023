package com.ragerobotics.frc2023.subsystems.wpilib;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ragerobotics.frc2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public static TalonSRX motor;
    private boolean mZeroed;

    public Arm() {
        motor = new TalonSRX(6);
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor.configForwardSoftLimitEnable(false);
        motor.config_kP(0, Constants.kArmKp);
        motor.config_kI(0, Constants.kArmKi);
        motor.config_kD(0, Constants.kArmKd);
        motor.config_kF(0, Constants.kArmKf);

        mZeroed = false;
    }

    @Override
    public void periodic() {
        if (!mZeroed && motor.isFwdLimitSwitchClosed() != 0) {
            motor.setSelectedSensorPosition(0);
            mZeroed = true;
        }
        if (mZeroed) {
            motor.setNeutralMode(NeutralMode.Brake);
        } else {
            motor.setNeutralMode(NeutralMode.Coast);
        }

        SmartDashboard.putNumber("Arm", motor.getSelectedSensorPosition());
    }
}
