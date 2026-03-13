// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

/** Add your docs here. */
public final class Constants {
    public static class OperatorConstants {
        public static final int kOperatorControllerPort = 0;
        public static final int kDriverControllerPort = 1;
    }

     public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds 
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.Meters.convertFrom(23, Inches);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.Meters.convertFrom(23, Inches);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Math.PI ;
    public static final double kFrontRightChassisAngularOffset = -Math.PI;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 3;   // Prev 3
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 9;

    public static final boolean kGyroReversed = false;
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

    public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.3;
    public static final double kPYController = 0.3;
    public static final double kPThetaController = 0.3;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  
    public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

    public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

    public static class ShooterSubsystemConstants {
        // Keep in mind this is for the MAIN FLYWHEEL
        public static final DCMotor dcMotor = DCMotor.getNeoVortex(2);
        public static final int canIDMaster = 10;
        public static final int canIDFollower = 12;
        public static final double kP = 40; // The actual value for P may be lower than 1
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 1;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double gearRatio = 12;
        // Limits11
        public static final AngularVelocity maxVelocityRPM = RPM.of(6000);
        public static final AngularAcceleration maxAccelerationRPM = RotationsPerSecondPerSecond.of(5000);
        public static final Current statorCurrentLimit = Amps.of(80);
        public static final Distance diameterWheel = Inches.of(4); // You can also use Centimeters.of(2.5) or something like that
        public static final Mass massWheel = Pounds.of(1);
        public static final AngularVelocity softLimitLower = RPM.of(-3000);
        public static final AngularVelocity softLimitUpper = RPM.of(7000);
    }

    public static class IntakeArmConstants {
        // public static final DCMotor dcMotor = DCMotor.getNeoVortex(1);
        // public static final int canIDMaster = 3;
        // public static final double kP = 1; 
        // public static final double kI = 0;
        // public static final double kD = 0;
        // public static final double kS = 0;
        // public static final double kV = 0;
        // public static final double kG = 0; // Gravity Constant
        // public static final double gearRatio = 12;
        // public static final boolean motorInverted = false;
        // // Limits
        // public static final Angle startingPositionAngle = Degrees.of(0);
        // public static final Angle softLimitLower = Degrees.of(-20);
        // public static final Angle softLimitUpper = Degrees.of(60);
        // public static final Angle hardLimitLower = Degrees.of(-30);
        // public static final Angle hardLimitUpper = Degrees.of(70);
        // public static final AngularVelocity maxVelocityDegPerSec = DegreesPerSecond.of(90);
        // public static final AngularAcceleration maxAccelerationDegPerSecPerSec = DegreesPerSecondPerSecond.of(45);
        // public static final Current statorCurrentLimit = Amps.of(40);
        // // Length and mass of arm
        // public static final Distance lengthArmFeet = Feet.of(3);
        // public static final Mass massArmPounds = Pounds.of(1);
    }

    public static class IntakeRollerConstants {
        // public static final DCMotor dcMotor = DCMotor.getNeoVortex(1);
        // // public static final int canID = 4;
        // public static final int canIDFollowerEAR = 5;
        // public static final double gearRatio = 12;
        // public static final boolean motorInverted = false;
        // // Limits
        // public static final Current statorCurrentLimit = Amps.of(40);
        // public static final AngularVelocity softLimitLower = RPM.of(-6000);
        // public static final AngularVelocity softLimitUpper = RPM.of(6000);
        // public static final Distance diameterWheel = Inches.of(4); // You can also use Centimeters.of(2.5) or something like that
        // public static final Mass massWheel = Pounds.of(1);
        // Roller Speed
        // public static final double feedCommandDutyCycle = 0.4;
        // public static final double backfeedCommandDutyCycle = -0.4;
    }

    public static class ShooterIntakeRollerConstants {
        // public static final DCMotor dcMotor = DCMotor.getNeoVortex(1);
        // public static final int canID = 6;
        // public static final double gearRatio = 12;
        // public static final boolean motorInverted = false;
        // // Limits
        // public static final Current statorCurrentLimit = Amps.of(40);
        // public static final AngularVelocity softLimitLower = RPM.of(0);
        // public static final AngularVelocity softLimitUpper = RPM.of(4000);
        // public static final Distance diameterWheel = Inches.of(4);
        // public static final Mass massWheel = Pounds.of(1);
    }

    public static class ClimbSubsystemConstants {
      public static final DCMotor dcMotor = DCMotor.getCIM(1);
      public static final int canID = 11;

        public static final double gearRatio = 100;
        public static final boolean motorInverted = false;
        // Limits
        public static final Current statorCurrentLimit = Amps.of(50);
}

    public static class IndexRollerConstants {
      public static final DCMotor dcMotor = DCMotor.getCIM(1);
      public static final int canID = 13;
      
      public static final double gearRatio = 12;
      public static final boolean motorInverted = false;
      
      public static final Current statorCurrentLimit = Amps.of(80);
    }
}
