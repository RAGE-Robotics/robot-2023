package com.ragerobotics.frc2023;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import com.ragerobotics.frc2023.commands.Intake.ConeHold;
import com.ragerobotics.frc2023.commands.Intake.IntakeIn;
import com.ragerobotics.frc2023.commands.Intake.IntakeOut;
import com.ragerobotics.frc2023.commands.LEDs.SignalCone;
import com.ragerobotics.frc2023.commands.LEDs.SignalCube;
// import com.ragerobotics.frc2023.commands.Intake.IntakeIn;
// import com.ragerobotics.frc2023.commands.Intake.IntakeOut;
// import com.ragerobotics.frc2023.commands.intake.IntakeOut;
import com.ragerobotics.frc2023.commands.arm.DoubleStationCube;
import com.ragerobotics.frc2023.commands.arm.GroundPos;
import com.ragerobotics.frc2023.commands.arm.ScoreCube;
import com.ragerobotics.frc2023.commands.arm.ScoreHorizontalCone;
import com.ragerobotics.frc2023.commands.arm.ScoreVerticalCone;
import com.ragerobotics.frc2023.commands.arm.SingleStation;
import com.ragerobotics.frc2023.commands.arm.StowedPos;

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
        mOperatorController.button(7).whileTrue(new ConeHold());
        mOperatorController.button(2).whileTrue(new GroundPos());
        mOperatorController.button(3).whileTrue(new DoubleStationCube());
        mOperatorController.povDown().whileTrue(new ScoreCube());
        mOperatorController.povUp().whileTrue(new ScoreVerticalCone());
        mOperatorController.povLeft().whileTrue(new ScoreHorizontalCone());
        mOperatorController.button(4).whileTrue(new StowedPos());
        mOperatorController.button(1).whileTrue(new SingleStation());

        mDriverController.button(5).whileTrue(new SignalCube());
        mDriverController.button(6).whileTrue(new SignalCone());
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
