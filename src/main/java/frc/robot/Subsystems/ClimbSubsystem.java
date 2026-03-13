// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbSubsystemConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimbSubsystem extends SubsystemBase {

  private SparkFlex cimMotor;
  private SmartMotorControllerConfig smcConfig;
  private SmartMotorController sparkSmartMotorController;
  private ArmConfig armCfg;
  // Arm Mechanism
  private Arm arm;

  public ClimbSubsystem() {
    cimMotor = new SparkFlex(ClimbSubsystemConstants.canID, MotorType.kBrushed);


    smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
     // Telemetry name and verbosity level
      .withTelemetry("cimMotor", TelemetryVerbosity.HIGH)
     // Gearing from the motor rotor to final shaft.
     // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
     // You could also use .withGearing(12) which does the same thing.
      .withGearing(ClimbSubsystemConstants.gearRatio)
      // Motor properties to prevent over currenting.
     .withMotorInverted(ClimbSubsystemConstants.motorInverted)
     .withIdleMode(MotorMode.BRAKE)
     .withStatorCurrentLimit(ClimbSubsystemConstants.statorCurrentLimit)
     .withOpenLoopRampRate(Seconds.of(0.25));

    sparkSmartMotorController = new SparkWrapper(cimMotor, ClimbSubsystemConstants.dcMotor, smcConfig);

    armCfg = new ArmConfig(sparkSmartMotorController)
     // Telemetry name and verbosity for the arm.
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    arm = new Arm(armCfg);
  }



  public Command set(double dutycycle) {
    return arm.set(dutycycle);
  }

  public Command stopMotor(double dutycycle) {
    return arm.set(0);
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
  }
}
