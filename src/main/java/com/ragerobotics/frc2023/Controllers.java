package com.ragerobotics.frc2023;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

public class Controllers {
    private static Controllers mInstance = null;

    public static Controllers getInstance() {
        if (mInstance == null)
            mInstance = new Controllers();
        return mInstance;
    }

    private Joystick mLeftJoystick;
    private Joystick mRightJoystick;
    private XboxController mDriverController;
    private PS4Controller mOperatorController;

    private Controllers() {
        mLeftJoystick = new Joystick(0);
        mRightJoystick = new Joystick(1);
        mDriverController = new XboxController(4);
        mOperatorController = new PS4Controller(5);
    }

    public Joystick getLeftJoystick() {
        return mLeftJoystick;
    }

    public Joystick getRightJoystick() {
        return mRightJoystick;
    }

    public XboxController getDriverController() {
        return mDriverController;
    }

    public PS4Controller getOperatorController() {
        return mOperatorController;
    }
}
