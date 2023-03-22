package com.ragerobotics.frc2023.subsystems.wpilib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ragerobotics.frc2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public static TalonSRX murphyArm = new TalonSRX(6);
    private boolean mZeroed;
    

    public Arm() {
        
        murphyArm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        murphyArm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        murphyArm.configForwardSoftLimitEnable(false);
        murphyArm.config_kP(0, Constants.kArmKp);
        murphyArm.config_kI(0, Constants.kArmKi);
        murphyArm.config_kD(0, Constants.kArmKd);
        murphyArm.config_kF(0, Constants.kArmKf);

        mZeroed = false;
    }

    public void coast(){
        murphyArm.setNeutralMode(NeutralMode.Coast);
    }

    public void brake(){
        murphyArm.setNeutralMode(NeutralMode.Brake);
    }

    public void maunelMode(double maunelPower){
        murphyArm.set(ControlMode.PercentOutput, maunelPower);
    }

    public static double getMurphyEncoder(){
        return murphyArm.getSelectedSensorPosition();
    }

    public void resetMaunelArm(){
        murphyArm.setSelectedSensorPosition(0);
    }

    public void positionMaunel(double position){
        murphyArm.set(ControlMode.Position, position);
    }

    @Override
    public void periodic() {
        if (!mZeroed && murphyArm.isFwdLimitSwitchClosed() != 0) {
            murphyArm.setSelectedSensorPosition(0);
            mZeroed = true;
        }
        if (mZeroed) {
            murphyArm.setNeutralMode(NeutralMode.Brake);
        } else {
            murphyArm.setNeutralMode(NeutralMode.Coast);
        }

        SmartDashboard.putNumber("Arm", murphyArm.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Percentage", murphyArm.getMotorOutputPercent());
    }

   
}
