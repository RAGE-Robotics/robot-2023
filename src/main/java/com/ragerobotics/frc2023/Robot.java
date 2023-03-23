package com.ragerobotics.frc2023;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ragerobotics.frc2023.commands.Drive.RAGEArcade;
import com.ragerobotics.frc2023.paths.TrajectoryGenerator;
import com.ragerobotics.frc2023.subsystems.Drive;
import com.ragerobotics.frc2023.subsystems.RobotStateEstimator;
import com.ragerobotics.frc2023.subsystems.wpilib.Arm;
import com.ragerobotics.frc2023.subsystems.wpilib.DriveTrain;
import com.ragerobotics.frc2023.subsystems.wpilib.Intake;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    public static TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final Looper mEnabledLooper = new Looper(Constants.kLooperDt);
    private final Looper mDisabledLooper = new Looper(Constants.kLooperDt);
    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();

    //Autonomous chooser
    private static final String kDefaultAuto = "Do Nothing";
    private static final String kDriveStraight = "Drive Straight";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // subsystems
    public static Drive mDrive = Drive.getInstance();
    public static DriveTrain driveTrain = new DriveTrain();
    public static Arm mArm = new Arm();
    public static Intake mIntake = new Intake();

    public static Controllers mControllers;

    @Override
    public void robotInit() {

        mControllers = new Controllers();
        mTrajectoryGenerator.generateTrajectories();
        mDrive.zeroSensors();


        // Auto Subsystem manager
        mSubsystemManager.setSubsystems(mRobotStateEstimator, mDrive);
        mSubsystemManager.registerEnabledLoops(mEnabledLooper);
        mSubsystemManager.registerDisabledLoops(mDisabledLooper);
        mSubsystemManager.stop();

        // Compressor stuff
        Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableAnalog(Constants.kCompressorMinPressure, Constants.kCompressorMaxPressure);
        compressor.close();

        // Setting Default Commands for Subsystems
        driveTrain.setDefaultCommand(new RAGEArcade());

        // autonomous options
        m_chooser.setDefaultOption(kDefaultAuto, kDefaultAuto);
        m_chooser.addOption(kDriveStraight, kDriveStraight);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    @Override
    public void robotPeriodic() {
        // mSubsystemManager.outputToSmartDashboard();

        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        mDisabledLooper.stop();
        mSubsystemManager.stop();
        mEnabledLooper.start();

        m_autoSelected = m_chooser.getSelected();

        final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory = new
        TrajectoryIterator<>(
        new TimedView<>(mTrajectoryGenerator.getTrajectorySet().testTrajectory));
        mRobotState.reset(Timer.getFPGATimestamp(),
        trajectory.getState().state().getPose());
        mDrive.setTrajectory(trajectory);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        mDisabledLooper.start();
        mSubsystemManager.stop();
        mEnabledLooper.stop();
        mDisabledLooper.stop();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        // mEnabledLooper.stop();
        // mDisabledLooper.start();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
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
