package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;

import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSTalonFXSwerveConstants;

public final class Constants {
  public static final double stickDeadband = 0.1;

  public static final class Controller {
    public static final int USB_DRIVECONTROLLER = 0;
    public static final int USB_AUXCONTROLLER = 1;
  }

  public static final class Targeting {

    public static final String[] CAMERA_NAMES = {
        "limelight",
        "limelight-back"
    };

    public static final double CHASSIS_WIDTH_INCHES = 27.5;
    public static final double CHASSIS_WIDTH_METERS = Units.inchesToMeters(CHASSIS_WIDTH_INCHES);
    public static final double ROBOT_WIDTH_INCHES = 33.5;
    public static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(ROBOT_WIDTH_INCHES);
    public static final double BLUE_TOWER_CENTER_Y_METERS = 3.746;
    public static final double RED_TOWER_CENTER_Y_METERS = 4.324;

    public static final double AUTO_ROTATE_KP = 4.25;
    public static final double AUTO_ROTATE_KD = 0.025;
    public static final double AUTO_ROTATE_FEEDFORWARD = 0.5;
    public static final double AUTO_ROTATE_TOLERANCE = 2.0;

    public static final double AUTO_SHAKE_KP = 90.0;
    public static final double AUTO_SHAKE_KD = 0.0;

  }

  public static final class PathPlanner {
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(10, 0.0, 0.0);
    public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(4, 0.0, 0.2); 
  }

  public static final class Swerve {
    public static final int pigeonID = 1;

    public static final COTSTalonFXSwerveConstants chosenModule =
       
        COTSTalonFXSwerveConstants.WCP.SwerveXFlipped
            .KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X2_11);

    public static final double trackWidth = Units.inchesToMeters(20.5);
    public static final double wheelBase = Units.inchesToMeters(20.5);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    public static final int angleCurrentLimit = 25; // Look into
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35; // Look into
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double angleKP = 100.0;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.05;
    public static final double angleKS = 0.05;
    public static final double angleKV = 1.5;

    public static final double driveKP = 2.5;
    public static final double driveKI = 0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    public static final double driveKS = 0;
    public static final double driveKV = 0;
    public static final double driveKA = 0;

    public static final double maxSpeed = 4.5;
    public static final double throttle = 1.0;
    public static final double maxAngularVelocity = 10.0;

    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;


    public static final class Mod0 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(40.78);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-175.34);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(67.32);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    public static final class Mod3 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-74.44);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4 * Math.PI;
    public static final double kPXController = 4;
    public static final double kPYController = 6;
    public static final double kPThetaController = 10;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static class XboxController {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int VIEW = 7;
    public static final int MENU = 8;
    public static final int LM = 9;
    public static final int RM = 10;

    public static class AxesXbox {
      public static final int LX = 0;
      public static final int LY = 1;
      public static final int LTrig = 2;
      public static final int RTrig = 3;
      public static final int RX = 4;
      public static final int RY = 5;
    }

    public class POVXbox {
      public static final int UP_ANGLE = 0;
      public static final int RIGHT_ANGLE = 90;
      public static final int DOWN_ANGLE = 180;
      public static final int LEFT_ANGLE = 270;
    }
  }
}

/*
 * ROBO CONSTANTS FOR PATHPLANNER
 * Width: 27.5in, 0.6985m
 * Length: 27.5in, 0.6985m
 * LLForward: -8.5in, -0.2159m
 * LLSideways: 0in, 0m
 * LLUp: 20.5in, 0.5207m
 * LLPitch: 24deg
 */