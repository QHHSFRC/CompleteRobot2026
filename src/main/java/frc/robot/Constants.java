// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
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

    public static class ShooterSubsystemConstants {
        // Keep in mind this is for the MAIN FLYWHEEL
        public static final DCMotor dcMotor = DCMotor.getNeoVortex(2);
        public static final int canIDMaster = 1;
        public static final int canIDFollower = 2;
        public static final double kP = 1; // The actual value for P may be lower than 1
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double gearRatio = 12;
        // Limits
        public static final AngularVelocity maxVelocityRPM = RPM.of(5000);
        public static final AngularAcceleration maxAccelerationRPM = RotationsPerSecondPerSecond.of(2500);
        public static final Current statorCurrentLimit = Amps.of(40);
        public static final Distance diameterWheel = Inches.of(4); // You can also use Centimeters.of(2.5) or something like that
        public static final Mass massWheel = Pounds.of(1);
        public static final AngularVelocity softLimitLower = RPM.of(0);
        public static final AngularVelocity softLimitUpper = RPM.of(6000);
    }

    public static class IntakeArmConstants {
        public static final DCMotor dcMotor = DCMotor.getNeoVortex(1);
        public static final int canIDMaster = 3;
        public static final double kP = 1; 
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kG = 0; // Gravity Constant
        public static final double gearRatio = 12;
        public static final boolean motorInverted = false;
        // Limits
        public static final Angle startingPositionAngle = Degrees.of(0);
        public static final Angle softLimitLower = Degrees.of(-20);
        public static final Angle softLimitUpper = Degrees.of(60);
        public static final Angle hardLimitLower = Degrees.of(-30);
        public static final Angle hardLimitUpper = Degrees.of(70);
        public static final AngularVelocity maxVelocityDegPerSec = DegreesPerSecond.of(90);
        public static final AngularAcceleration maxAccelerationDegPerSecPerSec = DegreesPerSecondPerSecond.of(45);
        public static final Current statorCurrentLimit = Amps.of(40);
        // Length and mass of arm
        public static final Distance lengthArmFeet = Feet.of(3);
        public static final Mass massArmPounds = Pounds.of(1);
    }

    public static class IntakeRollerConstants {
        public static final DCMotor dcMotor = DCMotor.getNeoVortex(1);
        public static final int canID = 4;
        public static final int canIDFollowerEAR = 5;
        public static final double gearRatio = 12;
        public static final boolean motorInverted = false;
        // Limits
        public static final Current statorCurrentLimit = Amps.of(40);
        public static final AngularVelocity softLimitLower = RPM.of(-6000);
        public static final AngularVelocity softLimitUpper = RPM.of(6000);
        public static final Distance diameterWheel = Inches.of(4); // You can also use Centimeters.of(2.5) or something like that
        public static final Mass massWheel = Pounds.of(1);
        // Roller Speed
        // public static final double feedCommandDutyCycle = 0.4;
        // public static final double backfeedCommandDutyCycle = -0.4;
    }
}
