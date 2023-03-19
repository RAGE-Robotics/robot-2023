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
    private static XboxController mDriverController;
    private PS4Controller mOperatorController;

    private Controllers() {
        mLeftJoystick = new Joystick(0);
        mRightJoystick = new Joystick(1);
        mDriverController = new XboxController(4);
        mOperatorController = new PS4Controller(5);
        configureBindings();
    }

    private void configureBindings() {
        // <--driver controller -->.<-- Button --> (<-- Insert Command here -->)

        // m_driverController.a().whileTrue(new IntakeIn());

    }

    // RAGEDrive Methods
    public static double GetTurn() {
        return mDriverController.getRawAxis(0);
    }
    
    public static double GetThrottle() {
        double reverse = mDriverController.getRawAxis(2);
        double forward = mDriverController.getRawAxis(3);

        if((reverse > .1) && (forward > .1)) {
            return 0;
        }
        else if(forward > .1) {
            return forward;
        }

        else if (reverse > .1) {
            return -reverse;
        }
        else {
            return 0;
        }
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
