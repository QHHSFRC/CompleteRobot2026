// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.ShooterIntakeRollerConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterRollerSubsystem extends SubsystemBase {

  private SparkFlex masterMotor;

  private SmartMotorControllerConfig smcConfig;
  private SmartMotorController smc;
  private final FlyWheelConfig rollerConfig;
  private FlyWheel roller;


  public ShooterRollerSubsystem() {
    masterMotor = new SparkFlex(ShooterIntakeRollerConstants.canID, MotorType.kBrushless);

    smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
    .withGearing(ShooterIntakeRollerConstants.gearRatio)
    .withMotorInverted(ShooterIntakeRollerConstants.motorInverted)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(ShooterIntakeRollerConstants.statorCurrentLimit);

    smc = new SparkWrapper(masterMotor, ShooterIntakeRollerConstants.dcMotor, smcConfig);

    rollerConfig = new FlyWheelConfig(smc)
    .withDiameter(ShooterIntakeRollerConstants.diameterWheel)
    .withMass(ShooterIntakeRollerConstants.massWheel)
    .withSoftLimit(ShooterIntakeRollerConstants.softLimitLower, ShooterIntakeRollerConstants.softLimitUpper)
    .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

    roller = new FlyWheel(rollerConfig);
  }

  public Command feed(double dutycycle) {
    return roller.set(dutycycle);
  }


  @Override
  public void periodic() {
    roller.updateTelemetry();
  }
}
