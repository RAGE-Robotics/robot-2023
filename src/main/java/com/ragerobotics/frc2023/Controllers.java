package com.ragerobotics.frc2023;

import edu.wpi.first.wpilibj.Joystick;

public class Controllers {
    private static Controllers mInstance = null;

    public static Controllers getInstance() {
        if (mInstance == null)
            mInstance = new Controllers();
        return mInstance;
    }

    private Joystick mLeftJoystick;
    private Joystick mRighJoystick;

    private Controllers() {
        mLeftJoystick = new Joystick(0);
        mRighJoystick = new Joystick(1);
    }

    public Joystick getLeftJoystick() {
        return mLeftJoystick;
    }

    public Joystick getRightJoystick() {
        return mRighJoystick;
    }
}
