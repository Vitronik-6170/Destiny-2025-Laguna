// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final int kMechanismsControllerPort = 1;
  }
  public static class LiftConstants {
    public static final int kLeftLiftMotorID = 11;
    public static final int kRightLiftMotorID = 10;
    public static final double kLiftPower = 0.8;

    public static final double kLiftFloor = 0;

    public static final double kLEFTLiftReef_L1 = 0;
    public static final double kLEFTLiftReef_L2 = 0;
    public static final double kLEFTLiftReef_L3 = 74.28549194335;
    public static final double kLEFTLiftAlgae = 50.85717391967;
    public static final double kLEFTLiftTop = 85.57318878173;

    public static final double kRIGHTLiftReef_L1 = 0;
    public static final double kRIGHTLiftReef_L2 = 0;
    public static final double kRIGHTLiftReef_L3 = 74.04737472534;
    public static final double kRIGHTLiftAlgae = 50.54761886596;
    public static final double kRIGHTLiftTop = 85.57318878173;    
  }
  public static class IntakeConstants {
    public static final int kIntakeMotorID = 15;
    public static final double kIntakePower = 0.9;
  }
  public static class WristConstants {
    public static final int kWristMotorID = 16;
    public static final double kWristEncoderOffset = 0.9745841;
    public static final double kWristPower = 0.3;

    public static final double verticalWrist = Math.PI;
    public static final double horizontalWrist = 3* Math.PI/2;
    public static final double FloorWrist = Math.PI/2;

    public static final double kWristReeft_L1 = FloorWrist;
    public static final double kWristReeft_L2 = verticalWrist;
    public static final double kWristReeft_L3 = verticalWrist;

  }
  public static class ArmConstants {
    public static final int kArmMotorID = 14;
    public static final double kArmEncoderOffset = 0.71;
    public static final double kArmPower = 0.8;

    public static final double kArmOut = 3;

    public static final double kArmFloor = 1.2;
    public static final double kArmHuman = 2.238913307189941+(Math.PI/2);

    public static final double kArmReef_L1 = 3.589863929748-(Math.PI/2);
    public static final double kArmReef_L2 = 3.838535308837-(Math.PI/2);
    public static final double kArmReef_L3 = 3.740459442138-(Math.PI/2);
  }
  public static class CageConstants {
    public static final int kLeftCageMotorID = 12;
    public static final int kRightCageMotorID = 13;
    public static final double kCagePower = 0.1;

    public static final double kCageEncoderOffset = 0.5;
    public static final double kCageInit = 5.28;
    public static final double kCagePrepareToHang = 1.75;
    public static final double kCageHang =3.577;
    public static final double kCageMaxHang = 5.28;
    public static final double kCageMinHang = 1.26;

    
    public static final double kKaleb = 7;
    public static final double kRight_PowerCageHang = 0.9;
    public static final double kLeft_PowerCageHang = 0.9;

  }
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 9;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 3;

    // CANCODER IDs
    public static final int kFrontLeftEncoderID = 1;
    public static final int kFrontRightEncoderID = 2;
    public static final int kRearLeftEncoderID = 3;
    public static final int kRearRightEncoderID = 4;

    // CANCODER OFFSETS
    public static final double kOffsetFrontLeft = -0.35205078125;
    public static final double kOffsetFrontRight = -0.30126953125;
    public static final double kOffsetRearLeft = -0.419189453125;
    public static final double kOffsetRearRight = 0.438720703125;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.15;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  
}
