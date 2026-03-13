// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexRollerConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IndexRollerSubsystem extends SubsystemBase {
  // This is the subsystem (roller) that makes the plate inside the hopper bounce up and down. Idk the actual name for it.
  
  private final SparkFlex cimMotor;
  private SmartMotorControllerConfig smcConfig;
  private SmartMotorController smc;
  private final FlyWheelConfig rollerConfig;
  private FlyWheel roller;

  public IndexRollerSubsystem() {
    cimMotor = new SparkFlex(IndexRollerConstants.canID, MotorType.kBrushed);

    smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("HopperRollerMotor", TelemetryVerbosity.HIGH)
    .withGearing(IndexRollerConstants.gearRatio)
    .withMotorInverted(IndexRollerConstants.motorInverted)
    .withStatorCurrentLimit(IndexRollerConstants.statorCurrentLimit)
    .withIdleMode(MotorMode.COAST);

    smc = new SparkWrapper(cimMotor, IndexRollerConstants.dcMotor, smcConfig);

    rollerConfig = new FlyWheelConfig(smc)
    .withTelemetry("HopperRoller", TelemetryVerbosity.HIGH);

    roller = new FlyWheel(rollerConfig);
  }

  public Command set(double dutycycle) {
    return roller.set(dutycycle);
  }

  @Override
  public void periodic() {
    roller.updateTelemetry();
  }
}
