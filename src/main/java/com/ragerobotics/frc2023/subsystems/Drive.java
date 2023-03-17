package com.ragerobotics.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;
import com.ragerobotics.frc2023.Constants;
import com.ragerobotics.frc2023.RobotState;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.ragerobotics.frc2023.planners.DriveMotionPlanner;
import com.team254.lib.drivers.*;
import com.team254.lib.geometry.*;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashSet;
import java.util.Set;

public class Drive extends Subsystem {

    private static Drive mInstance;

    // hardware
    private final TalonFX mLeftMaster1, mRightMaster1, mLeftMaster2, mRightMaster2;
    // private final DoubleSolenoid mShifter;
    private final DriveSide mLeftSide;
    private final DriveSide mRightSide;

    // control states
    private DriveControlState mDriveControlState;
    private AHRS mNavX;

    // hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();

    private DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

    private int kHighGearPIDSlot = 0;
    private int kLowGearPIDSlot = 1;

    public synchronized static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    @SuppressWarnings("deprecation")
    private void configureTalon(TalonFX talon, boolean left, boolean main_encoder_talon) {
        // general
        talon.setInverted(!left);
        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);

        talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        talon.configOpenloopRamp(0.4, Constants.kLongCANTimeoutMs);
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        mLeftMaster1 = TalonFXFactory.createDefaultTalon(new CanDeviceId(Constants.kLeftDriveMaster1Id));
        configureTalon(mLeftMaster1, true, false);

        mLeftMaster2 = TalonFXFactory.createDefaultTalon(new CanDeviceId(Constants.kLeftDriveMaster2Id));
        configureTalon(mLeftMaster2, true, true);

        mRightMaster1 = TalonFXFactory.createDefaultTalon(new CanDeviceId(Constants.kRightDriveMaster1Id));
        configureTalon(mRightMaster1, false, false);

        mRightMaster2 = TalonFXFactory.createDefaultTalon(new CanDeviceId(Constants.kRightDriveMaster2Id));
        configureTalon(mRightMaster2, false, true);

        /*
         * mShifter = new DoubleSolenoid(Constants.kPCMId, PneumaticsModuleType.REVPH,
         * Constants.kShifterSolenoidId1,
         * Constants.kShifterSolenoidId2);
         */

        mLeftSide = new DriveSide(mLeftMaster1, mLeftMaster2);
        mRightSide = new DriveSide(mRightMaster1, mRightMaster2);

        mNavX = new AHRS();

        // force a solenoid message
        mIsHighGear = false;
        setHighGear(true);

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();

        resetEncoders();
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    public static class PeriodicIO {
        // real outputs
        public double left_demand;
        public double right_demand;

        // INPUTS
        public double timestamp;
        public double left_voltage;
        public double right_voltage;
        public int left_position_ticks; // using us digital encoder
        public int right_position_ticks; // us digital encoder
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms; // using Talon FX
        public int right_velocity_ticks_per_100ms; // Talon FX
        public double left_velocity_in_per_sec;
        public double right_velocity_in_per_sec;
        public Rotation2d gyro_heading = Rotation2d.identity();

        // OUTPUTS
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public Pose2d error = Pose2d.identity();
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(
                Pose2dWithCurvature.identity());
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.left_voltage = mLeftSide.getPrimaryDriveTalonFX().getMotorOutputVoltage();
        mPeriodicIO.right_voltage = mRightSide.getPrimaryDriveTalonFX().getMotorOutputVoltage();

        mPeriodicIO.left_position_ticks = (int) mLeftMaster1.getSelectedSensorPosition();
        mPeriodicIO.right_position_ticks = (int) mRightMaster1.getSelectedSensorPosition();

        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(-mNavX.getAngle()).rotateBy(mGyroOffset);

        mPeriodicIO.left_distance = rotationsToInches(mPeriodicIO.left_position_ticks * getRotationsPerTickDistance());
        mPeriodicIO.right_distance = rotationsToInches(
                mPeriodicIO.right_position_ticks * getRotationsPerTickDistance());

        mPeriodicIO.left_velocity_ticks_per_100ms = (int) mLeftSide.getPrimaryDriveTalonFX()
                .getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = (int) mRightSide.getPrimaryDriveTalonFX()
                .getSelectedSensorVelocity(0);

        mPeriodicIO.left_velocity_in_per_sec = getLeftLinearVelocity();
        mPeriodicIO.right_velocity_in_per_sec = getRightLinearVelocity();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftSide.getDriveTalons().forEach(t -> t.set(ControlMode.PercentOutput, mPeriodicIO.left_demand));
            mRightSide.getDriveTalons().forEach(t -> t.set(ControlMode.PercentOutput, mPeriodicIO.right_demand));
        } else if (mDriveControlState == DriveControlState.VELOCITY
                || mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            double kd = isHighGear() ? Constants.kDriveHighGearKd : Constants.kDriveLowGearKd;
            mLeftSide.getDriveTalons()
                    .forEach(t -> t.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                            mPeriodicIO.left_feedforward + kd * mPeriodicIO.left_accel / 1023.0));
            mRightSide.getDriveTalons()
                    .forEach(t -> t.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                            mPeriodicIO.right_feedforward + kd * mPeriodicIO.right_accel / 1023.0));
        }
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    stop();
                    setBrakeMode(true);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    switch (mDriveControlState) {
                        case OPEN_LOOP:
                            break;
                        case VELOCITY:
                            break;
                        case PATH_FOLLOWING:
                            updatePathFollower();
                            break;
                        default:
                            System.out.println("unexpected drive control state: " + mDriveControlState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesToRadians(double inches) {
        return inches * 2.0 / Constants.kDriveWheelDiameterInches;
    }

    public static double radiansToInches(double radians) {
        return radians / 2.0 * Constants.kDriveWheelDiameterInches;
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    /**
     * @param rad_s of the output
     * @return ticks per 100 ms of the talonfx encoder
     */
    private double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) / getRotationsPerTickVelocity() / 10.0;
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    /**
     * Configure talons for velocity control during teleop
     */
    public synchronized void setVelocity(DriveOutput output) {
        if (mDriveControlState != DriveControlState.VELOCITY) {
            setBrakeMode(true);
            System.out.println("switching to velocity");
            mDriveControlState = DriveControlState.VELOCITY;
            configureTalonPIDSlot();
        }

        mPeriodicIO.left_demand = radiansPerSecondToTicksPer100ms(output.left_velocity);
        mPeriodicIO.right_demand = radiansPerSecondToTicksPer100ms(output.right_velocity);
        mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
        mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
        mPeriodicIO.left_feedforward = Util.epsilonEquals(mPeriodicIO.left_demand, 0.0) ? 0.0
                : output.left_feedforward_voltage / 12.0;
        mPeriodicIO.right_feedforward = Util.epsilonEquals(mPeriodicIO.right_demand, 0.0) ? 0.0
                : output.right_feedforward_voltage / 12.0;
    }

    /**
     * Configure talons for following via the ramsete controller
     */
    public synchronized void setRamseteVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            configureTalonPIDSlot();
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveOutput output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setRamseteVelocity(
                        new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity),
                                radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0,
                                output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setRamseteVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public synchronized boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void configureTalonPIDSlot() {
        int desired_slot_idx = isHighGear() ? kHighGearPIDSlot : kLowGearPIDSlot;

        mLeftSide.getDriveTalons().forEach(t -> t.selectProfileSlot(desired_slot_idx, 0));
        mRightSide.getDriveTalons().forEach(t -> t.selectProfileSlot(desired_slot_idx, 0));
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            // Plumbed default high.
            // mShifter.set(wantsHighGear ? Value.kForward : Value.kReverse);
            configureTalonPIDSlot();
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean shouldEnable) {
        if (mIsBrakeMode != shouldEnable) {
            mIsBrakeMode = shouldEnable;
            NeutralMode mode = shouldEnable ? NeutralMode.Brake : NeutralMode.Coast;

            mLeftSide.getDriveTalons().forEach(t -> t.setNeutralMode(mode));
            mRightSide.getDriveTalons().forEach(t -> t.setNeutralMode(mode));
        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(-mNavX.getAngle()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    public synchronized void resetEncoders() {
        mLeftMaster1.setSelectedSensorPosition(0);
        mRightMaster1.setSelectedSensorPosition(0);

        mLeftSide.getAllMotors().forEach(t -> t.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs));
        mRightSide.getAllMotors().forEach(t -> t.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs));

        mPeriodicIO = new PeriodicIO();
    }

    public double getLeftEncoderDistance() {
        return mPeriodicIO.left_distance;
    }

    public double getRightEncoderDistance() {
        return mPeriodicIO.right_distance;
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10 * getRotationsPerTickVelocity());
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10 * getRotationsPerTickVelocity());
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return (Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity())) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public double getLeftOutputVoltage() {
        return mPeriodicIO.left_voltage;
    }

    public double getRightOutputVoltage() {
        return mPeriodicIO.right_voltage;
    }

    public double getAverageOutputVoltageMagnitude() {
        return (Math.abs(getLeftOutputVoltage()) + Math.abs(getRightOutputVoltage())) / 2.0;
    }

    /**
     * @return conversion factor where ticks * getEncoderTicksPerRotation() = wheel
     *         rotations
     */
    public double getRotationsPerTickVelocity() { // talonfx
        return isHighGear() ? Constants.kDriveRotationsPerTickHighGear : Constants.kDriveRotationsPerTickLowGear;
    }

    public double getRotationsPerTickDistance() { // us digital encoders
        return 1.0 / Constants.kDriveEncoderPPR;
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control,
        VELOCITY, // velocity control
        PATH_FOLLOWING
    }

    public enum ShifterState {
        FORCE_LOW_GEAR, FORCE_HIGH_GEAR
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        setHighGear(true);

        // Trigger write to TalonFXs.
        stop();
        writePeriodicOutputs();

        setBrakeMode(false);

        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        SmartDashboard.putNumber("Left Drive 1 Current", mLeftMaster1.getStatorCurrent());
        SmartDashboard.putNumber("Left Drive 2 Current", mLeftMaster2.getStatorCurrent());
        SmartDashboard.putNumber("Right Drive 1 Current", mRightMaster1.getStatorCurrent());
        SmartDashboard.putNumber("Right Drive 2 Current", mRightMaster2.getStatorCurrent());

        SmartDashboard.putNumber("Left Drive Demand", mPeriodicIO.left_demand);
        SmartDashboard.putNumber("Right Drive Demand", mPeriodicIO.right_demand);
        SmartDashboard.putNumber("Left Drive Feedforward", mPeriodicIO.left_feedforward);
        SmartDashboard.putNumber("Right Drive Feedforward", mPeriodicIO.right_feedforward);

        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    public static class DriveSide {
        @SuppressWarnings("unused")
        private final TalonFX driveOnlyA, driveOnlyB;
        private Set<TalonFX> onlyDrive = new HashSet<>();
        private Set<TalonFX> allMotors = new HashSet<>();

        public DriveSide(TalonFX driveOnlyA, TalonFX driveOnlyB) {
            this.driveOnlyA = driveOnlyA;
            this.driveOnlyB = driveOnlyB;

            allMotors.add(driveOnlyA);
            allMotors.add(driveOnlyB);

            onlyDrive.add(driveOnlyA);
            onlyDrive.add(driveOnlyB);
        }

        public TalonFX getPrimaryDriveTalonFX() {
            return driveOnlyA;
        }

        public Set<TalonFX> getDriveTalons() {
            return allMotors;
        }

        public Set<TalonFX> getAllMotors() {
            return allMotors;
        }
    }
}
