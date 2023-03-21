package com.ragerobotics.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ragerobotics.frc2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends RageSubsystem {
    private static Arm instance = null;

    public static Arm getInstance() {
        if (instance == null)
            instance = new Arm();
        return instance;
    }

    private final TalonSRX mArmMotor = new TalonSRX(6);
    private boolean mArmZeroed = false;

    private Arm() {
        mArmMotor.configFactoryDefault();
        mArmMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mArmMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        mArmMotor.config_kP(0, Constants.kArmKp);
        mArmMotor.config_kI(0, Constants.kArmKi);
        mArmMotor.config_kD(0, Constants.kArmKd);
        mArmMotor.config_kF(0, Constants.kArmKf);
        // mArmMotor.configMotionCruiseVelocity(Constants.kArmCruiseVelocity);
        // mArmMotor.configMotionAcceleration(Constants.kArmAcceleration);

    //     mArmMotor.setInverted(true);
    //     mArmMotor.setSensorPhase(true);
    }

    @Override
    public void update(double timestamp, char mode) {
        if (!mArmZeroed && mArmMotor.isFwdLimitSwitchClosed() != 0) {
            mArmMotor.setSelectedSensorPosition(0);
            mArmZeroed = true;
        }
        if (mArmZeroed) {
            mArmMotor.setNeutralMode(NeutralMode.Brake);
        } else {
            mArmMotor.setNeutralMode(NeutralMode.Coast);
        }

        if (mode == 'a' || mode == 't') {
            mArmMotor.set(ControlMode.Position, Constants.kArmCubePlaceHeightPosition);
        }

        SmartDashboard.putNumber("Arm", mArmMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Power", mArmMotor.getMotorOutputPercent());

    }
}
