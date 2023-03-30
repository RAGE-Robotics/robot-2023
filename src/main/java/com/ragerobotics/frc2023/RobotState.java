package com.ragerobotics.frc2023;

import com.team254.lib.geometry.Pose2d;
/*import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;*/
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;
/*import com.team254.lib.vision.AimingParameters;
import com.team254.lib.vision.GoalTracker;
import com.team254.lib.vision.GoalTracker.TrackReportComparator;
import com.team254.lib.vision.TargetInfo;*/
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*import java.util.ArrayList;
import java.util.List;*/
import java.util.Map;
//import java.util.Optional;

public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 100;

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 3. Turret frame: origin is the center of the turret.
     *
     * 4. Camera frame: origin is the center of the Limelight relative to the
     * turret.
     *
     * 5. Target frame: origin is the center of the vision target, facing outwards
     * along the normal.
     *
     * As a kinematic chain with 5 frames, there are 4 transforms of interest:
     *
     * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     *
     * 2. Vehicle-to-turret: Rotation measured by the turret encoder; translation is
     * constant.
     *
     * 3. Turret-to-camera: This is a constant (per camera).
     *
     * 4. Camera-to-target: Measured by the vision system.
     */

    // FPGATimestamp -> Pose2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;
    private double distance_driven_;

    // private GoalTracker goal_tracker_ = new GoalTracker();

    private RobotState() {
        reset(0.0, Pose2d.identity());
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
        distance_driven_ = 0.0;
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist2d displacement, Twist2d measured_velocity,
            Twist2d predicted_velocity) {
        distance_driven_ += displacement.dx;
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), displacement));
        vehicle_velocity_measured_ = measured_velocity;
        if (Math.abs(vehicle_velocity_measured_.dtheta) < 2.0 * Math.PI) {
            // Reject really high angular velocities from the filter.
            vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        } else {
            vehicle_velocity_measured_filtered_
                    .add(new Twist2d(vehicle_velocity_measured_.dx, vehicle_velocity_measured_.dy, 0.0));
        }
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public Pose2d getRobot() {
        return new Pose2d();
    }

    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());

        SmartDashboard.putString("Field To Robot", getLatestFieldToVehicle().getValue().toString());

        /*
         * if (getVehicleToVisionTarget(Timer.getFPGATimestamp()) != null) {
         * SmartDashboard.putString("Robot to Vision Target",
         * getVehicleToVisionTarget(Timer.getFPGATimestamp()).toString());
         * }
         */
    }
}
