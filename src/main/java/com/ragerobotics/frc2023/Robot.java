package com.ragerobotics.frc2023;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ragerobotics.frc2023.paths.TrajectoryGenerator;
import com.ragerobotics.frc2023.subsystems.Arm;
import com.ragerobotics.frc2023.subsystems.Drive;
import com.ragerobotics.frc2023.subsystems.Intake;
import com.ragerobotics.frc2023.subsystems.RobotStateEstimator;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.loops.Looper;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final Looper mEnabledLooper = new Looper(Constants.kLooperDt);
    private final Looper mDisabledLooper = new Looper(Constants.kLooperDt);
    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();

    // subsystems
    public static Drive mDrive = Drive.getInstance();
    public static Arm mArm = Arm.getInstance();
    public static Intake mIntake = Intake.getInstance();

    private final Controllers mControllers = Controllers.getInstance();

    @Override
    public void robotInit() {
        mTrajectoryGenerator.generateTrajectories();
        mDrive.zeroSensors();

        mSubsystemManager.setSubsystems(mRobotStateEstimator, mDrive);
        mSubsystemManager.registerEnabledLoops(mEnabledLooper);
        mSubsystemManager.registerDisabledLoops(mDisabledLooper);
        mSubsystemManager.stop();

        Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableAnalog(Constants.kCompressorMinPressure, Constants.kCompressorMaxPressure);
        compressor.close();
    }

    @Override
    public void robotPeriodic() {
        mSubsystemManager.outputToSmartDashboard();
    }

    @Override
    public void autonomousInit() {
        mDisabledLooper.stop();
        mSubsystemManager.stop();
        mEnabledLooper.start();

        final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory = new TrajectoryIterator<>(
                new TimedView<>(mTrajectoryGenerator.getTrajectorySet().testTrajectory));
        mRobotState.reset(Timer.getFPGATimestamp(), trajectory.getState().state().getPose());
        mDrive.setTrajectory(trajectory);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        double timestamp = Timer.getFPGATimestamp();
        mArm.update(timestamp, 'a');
        mIntake.update(timestamp, 'a');
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        mDisabledLooper.stop();
        mSubsystemManager.stop();
        mEnabledLooper.start();
    }

    void driveArcade() {
        double throttle = -mControllers.getDriverController().getLeftY();
        double steer = 0.75 * mControllers.getDriverController().getRightX();

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

        double timestamp = Timer.getFPGATimestamp();
        mArm.update(timestamp, 't');
        mIntake.update(timestamp, 't');
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        mEnabledLooper.stop();
        mDisabledLooper.start();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        double timestamp = Timer.getFPGATimestamp();
        mArm.update(timestamp, 'd');
        mIntake.update(timestamp, 'd');
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
