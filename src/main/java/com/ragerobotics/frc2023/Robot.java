package com.ragerobotics.frc2023;

import com.ragerobotics.frc2023.subsystems.Drive;
import com.ragerobotics.frc2023.subsystems.RobotStateEstimator;
import com.team254.lib.loops.Looper;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private final Looper mEnabledLooper = new Looper(Constants.kLooperDt);
    private final Looper mDisabledLooper = new Looper(Constants.kLooperDt);
    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final Drive mDrive = Drive.getInstance();

    private final Controllers mControllers = Controllers.getInstance();

    @Override
    public void robotInit() {
        CrashTracker.logRobotInit();
        try {
            mDrive.zeroSensors();

            mSubsystemManager.setSubsystems(mRobotStateEstimator, mDrive);
            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            mSubsystemManager.stop();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            SmartDashboard.putNumber("Timestamp", Timer.getFPGATimestamp());
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        CrashTracker.logAutoInit();
        try {
            mDisabledLooper.stop();
            mSubsystemManager.stop();
            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        CrashTracker.logTeleopInit();
        try {
            mDisabledLooper.stop();
            mSubsystemManager.stop();
            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void driveTank() {
        double left = -mControllers.getLeftJoystick().getY();
        double right = -mControllers.getRightJoystick().getY();

        if (Math.abs(left) < Constants.kJoystickDeadband)
            left = 0;
        if (Math.abs(right) < Constants.kJoystickDeadband)
            right = 0;

        boolean leftNegative = left < 0;
        boolean rightNegative = right < 0;

        left *= left;
        right *= right;

        if (leftNegative)
            left *= -1;
        if (rightNegative)
            right *= -1;

        mDrive.setOpenLoop(new DriveSignal(left, right));
    }

    void driveArcade() {
        double throttle = -mControllers.getDriverController().getLeftY();
        double steer = mControllers.getDriverController().getRightX();

        if (Math.abs(throttle) < Constants.kArcadeDriveDeadband)
            throttle = 0;
        if (Math.abs(steer) < Constants.kArcadeDriveDeadband)
            steer = 0;

        boolean throttleNegative = throttle < 0;
        boolean steerNegative = steer < 0;

        throttle *= throttle;
        steer *= steer;

        if (throttleNegative)
            throttle *= -1;
        if (steerNegative)
            steer *= -1;

        mDrive.setOpenLoop(new DriveSignal(throttle + steer, throttle - steer));
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        driveArcade();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        CrashTracker.logDisabledInit();
        try {
            mEnabledLooper.stop();
            mDisabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
