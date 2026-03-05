// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;

import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.SmartMotorController;
import edu.wpi.first.math.system.plant.DCMotor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;


public class IntakeArmSubsystem extends SubsystemBase {
  private SmartMotorControllerConfig smcConfig;

  private SparkFlex sparkMaster;
  

  private SmartMotorController sparkSmartMotorController;

  private ArmConfig armCfg;

  // Arm Mechanism
  private Arm arm;


  public IntakeArmSubsystem() {
    

    sparkMaster = new SparkFlex(IntakeArmConstants.canIDMaster, MotorType.kBrushless);
    

    smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(IntakeArmConstants.kP, IntakeArmConstants.kI, IntakeArmConstants.kD, IntakeArmConstants.maxVelocityDegPerSec, IntakeArmConstants.maxAccelerationDegPerSecPerSec)
      // .withSimClosedLoopController(0, 0, 0, DegreesPerSecond.of(300), DegreesPerSecondPerSecond.of(300))
      // Feedforward Constants
      .withFeedforward(new ArmFeedforward(IntakeArmConstants.kS, IntakeArmConstants.kG , IntakeArmConstants.kV))
     // .withSimFeedforward(new ArmFeedforward(0, 0, 0))
     // Telemetry name and verbosity level
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
     // Gearing from the motor rotor to final shaft.
     // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
     // You could also use .withGearing(12) which does the same thing.
      .withGearing(IntakeArmConstants.gearRatio)
      // Motor properties to prevent over currenting.
     .withMotorInverted(IntakeArmConstants.motorInverted)
     .withIdleMode(MotorMode.BRAKE)
     .withStatorCurrentLimit(IntakeArmConstants.statorCurrentLimit)
     .withClosedLoopRampRate(Seconds.of(0.25));

    sparkSmartMotorController = new SparkWrapper(sparkMaster, IntakeArmConstants.dcMotor, smcConfig);

    armCfg = new ArmConfig(sparkSmartMotorController)
      // Soft limit is applied to the SmartMotorControllers PID
      .withSoftLimits(IntakeArmConstants.softLimitLower, IntakeArmConstants.softLimitUpper)
      // Hard limit is applied to the simulation.
      .withHardLimit(IntakeArmConstants.hardLimitLower, IntakeArmConstants.hardLimitUpper)
      // Starting position is where your arm starts
      .withStartingPosition(IntakeArmConstants.startingPositionAngle)
      // Length and mass of your arm for sim.
      .withLength(IntakeArmConstants.lengthArmFeet)
      .withMass(IntakeArmConstants.massArmPounds)
     // Telemetry name and verbosity for the arm.
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    arm = new Arm(armCfg);

  }

  public Command setAngle(Angle angle) {
    return arm.run(angle);
  }

  public Command setAngleAndStop(Angle angle, Angle angleTolerance) {
    return arm.runTo(angle, angleTolerance);
  }

  public void setAngleSetpoint(Angle angle) {
    arm.setMechanismPositionSetpoint(angle);
  }

  public Command set(double dutycycle) {
    return arm.set(dutycycle);
  }



  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}
