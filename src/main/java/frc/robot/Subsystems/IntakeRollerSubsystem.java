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
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeRollerSubsystem extends SubsystemBase {

  // THIS SUBSYSTEM REFERS TO THE MULTIPLE LONG WHITE ROLLERS
  private SparkFlex motorMaster;
  private SparkFlex motorFollowerEndAffectorRoller; // This motor is the motor attached to the end affector of the IntakeArm.

  private SmartMotorControllerConfig smcConfig;
  private SmartMotorController smc;
  private final FlyWheelConfig rollerConfig;
  private FlyWheel roller;

  public IntakeRollerSubsystem() {
    motorMaster = new SparkFlex(IntakeRollerConstants.canID, MotorType.kBrushless);
    motorFollowerEndAffectorRoller = new SparkFlex(IntakeRollerConstants.canIDFollowerEAR, MotorType.kBrushless);

    smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
    .withGearing(IntakeRollerConstants.gearRatio)
    .withMotorInverted(IntakeRollerConstants.motorInverted)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(IntakeRollerConstants.statorCurrentLimit)
    .withFollowers(Pair.of(motorFollowerEndAffectorRoller, true));

    smc = new SparkWrapper(motorMaster, IntakeRollerConstants.dcMotor, smcConfig);

    rollerConfig = new FlyWheelConfig(smc)
    .withDiameter(IntakeRollerConstants.diameterWheel)
    .withMass(IntakeRollerConstants.massWheel)
    .withSoftLimit(IntakeRollerConstants.softLimitLower, IntakeRollerConstants.softLimitUpper)
    .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

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
