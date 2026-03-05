// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.Constants.ShooterSubsystemConstants;

public class ShooterSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig;

  // If you're using a sparkflex for the flywheel mechanism, use replace "SparkMax" with "SparkFlex" and import its dependency. Also, I'm assuming the shooter subsystem contains two motors
  private SparkFlex sparkMaster;
  private SparkFlex sparkFollower;


  private SmartMotorController sparkSmartMotorController;

  private final FlyWheelConfig shooterConfig;

  // Shooter Mechanism
  private FlyWheel shooter;

  public ShooterSubsystem() {

    sparkMaster = new SparkFlex(ShooterSubsystemConstants.canIDMaster, MotorType.kBrushless);
    sparkFollower = new SparkFlex(ShooterSubsystemConstants.canIDFollower, MotorType.kBrushless);

    smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withClosedLoopController(ShooterSubsystemConstants.kP, ShooterSubsystemConstants.kI, ShooterSubsystemConstants.kD, ShooterSubsystemConstants.maxVelocityRPM, ShooterSubsystemConstants.maxAccelerationRPM)
  // .withSimClosedLoopController(0.1, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
  .withFeedforward(new SimpleMotorFeedforward(ShooterSubsystemConstants.kS, ShooterSubsystemConstants.kV, ShooterSubsystemConstants.kA))
  // .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  .withGearing(ShooterSubsystemConstants.gearRatio)
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST) 
  .withStatorCurrentLimit(ShooterSubsystemConstants.statorCurrentLimit)
  .withFollowers(Pair.of(sparkFollower, true));


  sparkSmartMotorController = new SparkWrapper(sparkMaster, ShooterSubsystemConstants.dcMotor, smcConfig);

  // These values aren't necessarily important for us, later on they may be.
  shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
  .withDiameter(ShooterSubsystemConstants.diameterWheel) // You can also change this to "Centimeters.of(2.54)" or whatever you'd like
  .withMass(ShooterSubsystemConstants.massWheel) // Mass of flywheel, mainly for sim
  .withSoftLimit(ShooterSubsystemConstants.softLimitLower, ShooterSubsystemConstants.softLimitUpper) // rpm limits for mechanism
  .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  shooter = new FlyWheel(shooterConfig);

  }

    // Commands, reutrns current velocity
  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return shooter.run(speed);
  }

  public void setVelocitySetpoint(AngularVelocity speed) {
    shooter.setMechanismVelocitySetpoint(speed);
  }

  public Command set(double dutyCycle) {
    return shooter.set(dutyCycle);
  }

  @Override
  public void periodic() {
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }
}
