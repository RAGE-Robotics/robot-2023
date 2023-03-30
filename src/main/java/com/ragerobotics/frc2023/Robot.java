package com.ragerobotics.frc2023;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ragerobotics.frc2023.LimelightHelpers.LimelightResults;
import com.ragerobotics.frc2023.commands.Auto_commands.Balance;
import com.ragerobotics.frc2023.commands.Auto_commands.ConeAndBalance;
import com.ragerobotics.frc2023.commands.Auto_commands.ConeAndCross;
import com.ragerobotics.frc2023.commands.Auto_commands.CubeAndBalance;
import com.ragerobotics.frc2023.commands.Auto_commands.CubeAndCross;
import com.ragerobotics.frc2023.commands.Auto_commands.DoNothing;
import com.ragerobotics.frc2023.commands.Auto_commands.DoubleCube;
import com.ragerobotics.frc2023.commands.Auto_commands.DriveStraight;
import com.ragerobotics.frc2023.commands.Drive.RAGEArcade;
import com.ragerobotics.frc2023.commands.LEDs.AllianceColor;
import com.ragerobotics.frc2023.paths.TrajectoryGenerator;
import com.ragerobotics.frc2023.subsystems.Drive;
import com.ragerobotics.frc2023.subsystems.RobotStateEstimator;
import com.ragerobotics.frc2023.subsystems.wpilib.Arm;
import com.ragerobotics.frc2023.subsystems.wpilib.DriveTrain;
import com.ragerobotics.frc2023.subsystems.wpilib.Intake;
import com.ragerobotics.frc2023.subsystems.wpilib.LEDs;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.loops.Looper;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    // Autonomous chooser
    private static final String kDriveStraight = "Drive Straight";
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    // subsystems
    public static Drive mDrive = Drive.getInstance();
    public static DriveTrain driveTrain = new DriveTrain();
    public static Arm mArm = new Arm();
    public static Intake mIntake = new Intake();
    public static LEDs mleds = new LEDs();

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
        mleds.setDefaultCommand(new AllianceColor());

        // autonomous options
        m_chooser.setDefaultOption("Do Nothing", new DoNothing());
        m_chooser.addOption(kDriveStraight, new DriveStraight());
        m_chooser.addOption("Cube and Cross", new CubeAndCross());
        m_chooser.addOption("Cone and Cross", new ConeAndCross());
        m_chooser.addOption("Cube and Balance", new CubeAndBalance());
        m_chooser.addOption("Cone and Balance", new ConeAndBalance());
        m_chooser.addOption("Just balance", new Balance());
        m_chooser.addOption("Double Cube", new DoubleCube());
        SmartDashboard.putData(m_chooser);

    }

    @Override
    public void robotPeriodic() {
        mSubsystemManager.outputToSmartDashboard();
        SmartDashboard.putNumber("Pitch", mDrive.NAVXpitch());

        LimelightResults llresults = LimelightHelpers.getLatestResults("");
        if (llresults.targetingResults.targets_Fiducials.length > 0) {
            double x = llresults.targetingResults.botpose_wpiblue[0] * 100 / 2.54;
            double y = llresults.targetingResults.botpose_wpiblue[1] * 100 / 2.54;
            double z = llresults.targetingResults.botpose_wpiblue[2] * 100 / 2.54;
            double theta = llresults.targetingResults.botpose_wpired[5] / 360 * Math.PI * 2;

            if (Math.abs(z) < 24) {
                System.out.println("x: " + x + ", y: " + y + ", z: " + z + ", theta: " + theta);
            }
        }

        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        mDisabledLooper.stop();
        mSubsystemManager.stop();
        mEnabledLooper.start();
        m_autonomousCommand = m_chooser.getSelected();

        mArm.toggleBrake();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        mSubsystemManager.stop();
        mDisabledLooper.stop();
        mEnabledLooper.start();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
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
