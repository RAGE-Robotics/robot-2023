package com.ragerobotics.frc2023;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ragerobotics.frc2023.commands.Intake.IntakeIn;
import com.ragerobotics.frc2023.commands.Intake.IntakeOut;
// import com.ragerobotics.frc2023.commands.Intake.IntakeIn;
// import com.ragerobotics.frc2023.commands.Intake.IntakeOut;
// import com.ragerobotics.frc2023.commands.intake.IntakeOut;
import com.ragerobotics.frc2023.commands.arm.DoubleStationCube;

public class Controllers {
    private static Controllers mInstance = null;

    public static Controllers getInstance() {
        if (mInstance == null)
            mInstance = new Controllers();
        return mInstance;
    }

    private Joystick mLeftJoystick;
    private Joystick mRightJoystick;
    private static CommandXboxController mDriverController;
    private static CommandXboxController mOperatorController;

    public Controllers() {
        mLeftJoystick = new Joystick(0);
        mRightJoystick = new Joystick(1);
        mDriverController = new CommandXboxController(4);
        mOperatorController = new CommandXboxController(5);
        configureBindings();
    }

    private void configureBindings() {
        // <--driver controller -->.<-- Button --> (<-- Insert Command here -->)

        // operator controls
        mOperatorController.rightBumper().whileTrue(new IntakeOut());
        mOperatorController.leftBumper().whileTrue(new IntakeIn());
        // mOperatorController.rightBumper().whileTrue(new DoubleStationCube());
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
}
