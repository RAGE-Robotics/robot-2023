package com.ragerobotics.frc2023;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;


import com.ragerobotics.frc2023.commands.Intake.IntakeIn;
import com.ragerobotics.frc2023.commands.Intake.IntakeOut;
// import com.ragerobotics.frc2023.commands.Intake.IntakeIn;
// import com.ragerobotics.frc2023.commands.Intake.IntakeOut;
// import com.ragerobotics.frc2023.commands.intake.IntakeOut;
import com.ragerobotics.frc2023.commands.arm.DoubleStationCube;
import com.ragerobotics.frc2023.commands.arm.GroundPos;
import com.ragerobotics.frc2023.commands.arm.ScoreCube;

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
    private static CommandGenericHID mOperatorController;

    public Controllers() {
        mLeftJoystick = new Joystick(0);
        mRightJoystick = new Joystick(1);
        mDriverController = new CommandXboxController(4);
        mOperatorController = new CommandGenericHID(5);
        configureBindings();
    }

    private void configureBindings() {
        // <--driver controller -->.<-- Button --> (<-- Insert Command here -->)

        // operator controls
        mOperatorController.button(5).whileTrue(new IntakeIn());
        mOperatorController.button(6).whileTrue(new IntakeOut());
        mOperatorController.button(2).whileTrue(new GroundPos());
        mOperatorController.button(3).whileTrue(new DoubleStationCube());
        mOperatorController.povDown().whileTrue(new ScoreCube());
        // mOperatorController.rightBumper().whileTrue(new IntakeOut());
        // mOperatorController.leftBumper().whileTrue(new IntakeIn());
        // mOperatorController.x().whileTrue(new GroundPos()); //actually a
        // mOperatorController.b().whileTrue(new DoubleStationCube());
        // mOperatorController.a().whileTrue(new ScoreCube()); //actually x
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
