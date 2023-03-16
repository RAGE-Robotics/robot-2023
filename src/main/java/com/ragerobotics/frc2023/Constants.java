package com.ragerobotics.frc2023;

public class Constants {
    public static final double kDriveWheelDiameterInches = 5.9067052758; // tuned 3/2
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveLinearKv = 0.0438 / 2.0 * Constants.kDriveWheelDiameterInches; // V / rad/s
    public static final double kRobotLinearInertia = 62.051; // kg TODO
    public static final double kDriveHighGearReduction = 40.0 / 11.0 * 50.0 / 14.0;
    public static final double kGearRatioScalar = (1.0 / (40.0 / 10.0 * 50.0 / 14.0)) / (1.0 / kDriveHighGearReduction);
    public static final double kDriveLinearKa = 0.00597 / 2.0 * Constants.kDriveWheelDiameterInches * kGearRatioScalar; // V
                                                                                                                        // /
                                                                                                                        // rad/s^2
    public static final double kDriveVIntercept = 0.352; // V TODO
    public static final double kDriveAngularKa = 0.00517 / 2.0 * Constants.kDriveWheelDiameterInches * kGearRatioScalar; // V
                                                                                                                         // per
                                                                                                                         // rad/s^2
    public static final double kDriveWheelTrackWidthInches = 30.0; // tuned 3/2
    public static final double kDriveWheelTrackRadiusWidthMeters = kDriveWheelTrackWidthInches / 2.0 * 0.0254;
    public static final double kRobotAngularInertia = kDriveAngularKa / kDriveLinearKa *
            kDriveWheelTrackRadiusWidthMeters * kDriveWheelTrackRadiusWidthMeters * kRobotLinearInertia; // kg m^2
    public static final double kRobotAngularDrag = 15.0; // N*m / (rad/sec)
    public static final double kTrackScrubFactor = 1.0;
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches
    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    public static final double kDriveHighGearKp = 0.15;
    public static final double kDriveHighGearKi = 0.0;
    public static final double kDriveHighGearKd = 0.0;
    public static final double kDriveHighGearKf = 0.0;
    public static final double kDriveLowGearKp = 0.0;
    public static final double kDriveLowGearKi = 0.0;
    public static final double kDriveLowGearKd = 0.0;
    public static final double kDriveLowGearKf = 0.0;
    public static final double kDrivePositionKp = 0.006;
    public static final double kDrivePositionKi = 0.0;
    public static final double kDrivePositionKd = 0.0;
    public static final double kDrivePositionKf = 0.0;
    public static final int kLeftDriveMaster1Id = 14;
    public static final int kLeftDriveMaster2Id = 13;
    public static final int kRightDriveMaster1Id = 1;
    public static final int kRightDriveMaster2Id = 5;
    public static final int kPCMId = 0;
    public static final int kShifterSolenoidId1 = 7;
    public static final int kShifterSolenoidId2 = 5;
    public static final double kDriveLowGearReduction = 40.0 / 11.0 * 44.0 / 20.0;
    public static final double kDriveRotationsPerTickHighGear = 1.0 / 2048.0 * 1.0 / kDriveLowGearReduction; // ticks *
                                                                                                             // kDriveRotationsPerTicksHighGear
                                                                                                             // = wheel
                                                                                                             // rotations
    public static final double kDriveRotationsPerTickLowGear = 1.0 / 2048.0 * 1.0 / kDriveHighGearReduction; // ticks *
                                                                                                             // kDriveRotationsPerTicksLowGear
                                                                                                             // = wheel
                                                                                                             // rotations
    public static final double kDriveEncoderPPR = 1000.0;
}
