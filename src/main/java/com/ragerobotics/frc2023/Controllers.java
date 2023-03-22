package com.ragerobotics.frc2023;

import com.ragerobotics.frc2023.commands.intake.*;
// import com.ragerobotics.frc2023.commands.intake.IntakeIn;

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
    private static XboxController mOperatorController;

    public Controllers() {
        mLeftJoystick = new Joystick(0);
        mRightJoystick = new Joystick(1);
        mDriverController = new XboxController(4);
        mOperatorController = new XboxController(5);
        configureBindings();
    }

    private void configureBindings() {
        // <--driver controller -->.<-- Button --> (<-- Insert Command here -->)

        // m_driverController.a().whileTrue(new IntakeIn());
        //mOperatorController.leftBumper().whileTrue(new IntakeIn());
        // mOperatorController.a().whileTrue(new IntakeIn());
        // (new IntakeOut());
    }

    // RAGEDrive Methods
    public static double GetTurn() {
        return mDriverController.getRawAxis(0);
    }

    public static double GetThrottle() {
        double reverse = mDriverController.getRawAxis(2);
        double forward = mDriverController.getRawAxis(3);

        if ((reverse > .1) && (forward > .1)) {
            return 0;
        } else if (forward > .1) {
            return forward;
        }

        else if (reverse > .1) {
            return -reverse;
        } else {
            return 0;
        }
    }

    public static double getLeftDriverY() {
        return mDriverController.getLeftY();
    }

    public static double getRightDriverX() {
        return mDriverController.getRightX();
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

    public XboxController getOperatorController() {
        return mOperatorController;
    }
}
