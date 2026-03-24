// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.ShooterUtil;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOpControllerPort = 1;
    public static final int kPoseControllerPort = 2;
  }

  public static final class RobotConstants {
        public static final double robotOverallLength = Units.inchesToMeters(28);
        public static final double robotOverallWidth = Units.inchesToMeters(27);
    }


public static enum POIs{
   RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
    RED_OUTPOST(new Translation3d(15.75, 7.25, 0)),
    RED_FAR_SIDE(new Translation3d(15.75, 0.75, 0)),

    BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
    BLUE_OUTPOST(new Translation3d(0.75, 0.75, 0)),
    BLUE_FAR_SIDE(new Translation3d(0.75, 7.25, 0));

    public final Translation3d value;
    
    private POIs(Translation3d value){
      this.value = value;
    }
      public static final Translation3d getAllianceHubPosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_HUB.value : BLUE_HUB.value;
    }

    public static final Translation3d getAllianceOutpostPosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_OUTPOST.value : BLUE_OUTPOST.value;
    }

    public static final Translation3d getAllianceFarSidePosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_FAR_SIDE.value : BLUE_FAR_SIDE.value;
    }
}
  
  
  
public static class TalonIDs{
  public static final int IntakeSpin = 1;
  public static final int IntakeFlip = 2;
  public static final int HopperMotor = 3;
  public static final int kickerMotor = 4;
  public static final int shooterL = 5;
  public static final int shooterR = 6;
  public static final int turretmotor = 7;
  
    
  }



public static class ControllerConstants{
  public static final double DEADBAND =.1;
}
 
    public static final class DriveConstants {
        public static final double DEADBAND = 0.05;
        public static final double SPEED_MULTIPLIER = 100.0;
        public static final double ALIGN_PID_P = 0.4;
        public static final double ALIGN_PID_I = 0.0;
        public static final double ALIGN_PID_D = 0.00;

        public static final double ALIGN_KS = 0.04;
        public static final double ALIGN_TOLERANCE_DEG = 0.5;
        public static final double ALIGN_TOLERANCE_VEL_DEG_PER_SEC = 10.0;

        public static final double ALIGN_MAX_VELOCITY_DEG_PER_SEC = 250.0;
        public static final double ALIGN_MAX_ACCEL_DEG_PER_SEC_SQ = 300.0;

        public static final double AUTO_DRIVE_P = 10;
        public static final double AUTO_DRIVE_I = 0.0;
        public static final double AUTO_DRIVE_D = 0;

        public static final double AUTO_ROTATION_P = 7;
        public static final double AUTO_ROTATION_I = 0.0;
        public static final double AUTO_ROTATION_D = 0;
    }
   public static final class LimelightConstants {
        public static final Set<Integer> RED_HUB_TAGS = Set.of(11, 2, 9, 10, 8, 5);
        public static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 27, 26, 25, 21, 24);
        public static final String LIMELIGHT_NAME = "limelight-cyber";

        public static final boolean ENABLE_VISION_ODOMETRY = true;
        public static final boolean TEST_MODE = false;
        public static final double VISION_REJECTION_SPEED_THRESHOLD_MPS = 2.0;
        public static final double VISION_REJECTION_DISTANCE_THRESHOLD_METERS = 1.0;
        public static final int PDHCANID = 24;
    }
     public static final class ShooterConstants {
        public static final int leftShooterId = 17;
        public static final int leftFeederId = 18;
        public static final int rightShooterId = 20;
        public static final int rightFeederId = 19;
        public static final double feederVoltage = 8.0; //8.0
        public static final double autonfeederVoltage = 7.0; 
        public static final double feederCurrentLimit = 20.0;
        public static final int leftWasherMotorId = 16;
        public static final int rightWasherMotorId = 15;
        public static final double shooterVelocityRPS = 65;
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.12;
        public static final double washerVoltage = 6.0; //6.0
        public static final double autonWasherVoltage = 5.0; //4.0
        public static final double shooterTargetVoltage = 12.0;
        public static final double shooterSpeedToleranceRPS = 2.0; 
        public static final double shootAtDistanceDurationSeconds = 3.0;
        public static final double shooterCurrentLimit = 40.0;
        public static final double rpsAdjustmentDelta = 2.5;

        public static final InterpolatingDoubleTreeMap distanceToVelocityRPS = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToVoltage = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToTimeOfFlight = new InterpolatingDoubleTreeMap();

        public static final double SHOOTER_KV_RPS_PER_VOLT = ShooterUtil.SHOOTER_KV_RPS_PER_VOLT;

        static {
            ShooterUtil.populateShotDataMaps(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight);
        }
    }
    
    public static final class FieldConstants {

        static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltAndymark);
        public static final double fieldLength = aprilTagFieldLayout.getFieldLength();
        public static final double fieldWidth = aprilTagFieldLayout.getFieldWidth();

        public static final Distance ALLIANCE_ZONE = Inches.of(156.06);
         public static final Distance FIELD_LENGTH = Inches.of(650.12);
        public static final Distance FIELD_WIDTH = Inches.of(316.64);

        static final double blueStartingLineX = Units.inchesToMeters(156);
        static final double blueOutpostSideTrenchStartY = Units.inchesToMeters(24.85);
        static final double blueDepotSideTrenchStartY = fieldWidth - Units.inchesToMeters(24.85);
        static final double bumpWidth = Inches.of(73.0).in(Meters);
        static final double bumpDepth = Inches.of(44.4).in(Meters);

        public static final Pose2d blueOutpostSideTrenchStart = new Pose2d(
                blueStartingLineX - RobotConstants.robotOverallLength / 2.0,
                blueOutpostSideTrenchStartY,
                new Rotation2d());

        public static final Pose2d blueDepotSideTrenchStart = new Pose2d(
                blueStartingLineX - RobotConstants.robotOverallLength / 2.0,
                blueDepotSideTrenchStartY,
                new Rotation2d());

        public static final Pose2d blueCenterStart = new Pose2d(
                blueStartingLineX - RobotConstants.robotOverallLength / 2.0,
                fieldWidth / 2.0,
                new Rotation2d());

        static final double hubCenterFromAllianceWall = Units.inchesToMeters(158.6);
        static final double bumpCenterFromAllianceWall = Units.inchesToMeters(95.25);
        static final double hubToBumpCenterOffset = Units.inchesToMeters(90.0);

        public static Pose2d blueHubPose = new Pose2d(
               Units.inchesToMeters(172), 
               Units.inchesToMeters(158), 
               new Rotation2d());

        static double redStartingLineX = Units.inchesToMeters(fieldLength - Units.inchesToMeters(143.5));

        public static Pose2d redHubPose = new Pose2d(
                Units.inchesToMeters(458),
                Units.inchesToMeters(158),
                new Rotation2d(Math.PI));

        public static Pose2d blueTowerRightPose = new Pose2d(
                blueHubPose.getX() - Units.inchesToMeters(151),
                blueHubPose.getY() - Units.inchesToMeters(45),
                new Rotation2d());

        public static Pose2d redTowerRightPose = new Pose2d(
                redHubPose.getX() + Units.inchesToMeters(151),
                redHubPose.getY() + Units.inchesToMeters(45),
                new Rotation2d(Math.PI));

        public static final Pose2d blueBumpLeftPose = new Pose2d(
                blueHubPose.getX(),
                blueHubPose.getY() + hubToBumpCenterOffset + Units.inchesToMeters(2.5),
                new Rotation2d());

        public static final Pose2d blueBumpRightPose = new Pose2d(
                blueHubPose.getX(),
                blueHubPose.getY() - hubToBumpCenterOffset,
                new Rotation2d());

        public static final Pose2d redBumpLeftPose = new Pose2d(
                redHubPose.getX(),
                redHubPose.getY() - hubToBumpCenterOffset,
                new Rotation2d(Math.PI));

        public static final Pose2d redBumpRightPose = new Pose2d(
                redHubPose.getX(),
                redHubPose.getY() + hubToBumpCenterOffset + Units.inchesToMeters(2.5),
                new Rotation2d(Math.PI));

    }

}

