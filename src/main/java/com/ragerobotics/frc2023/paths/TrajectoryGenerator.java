package com.ragerobotics.frc2023.paths;

import com.ragerobotics.frc2023.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    // TODO tune
    private static final double kMaxVel = 150.0;
    private static final double kMaxAccel = 100.0;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel, // inches/s
            double end_vel, // inches/s
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel,
                max_accel, max_voltage);
    }

    public class TrajectorySet {
        public final Trajectory<TimedState<Pose2dWithCurvature>> driveStraight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> driveBalance;

        private TrajectorySet() {
            driveStraight = driveStraight();
            driveBalance = driveBalance();
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> driveStraight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(156, 0, Rotation2d.fromDegrees(0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> driveBalance() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(84, 0, Rotation2d.fromDegrees(0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVel, kMaxAccel, kMaxVoltage);
        }
    }
}
