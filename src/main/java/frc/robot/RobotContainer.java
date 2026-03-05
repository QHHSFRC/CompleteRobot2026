// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterIntakeRollerConstants;
import frc.robot.Subsystems.IntakeArmSubsystem;
import frc.robot.Subsystems.IntakeRollerSubsystem;
import frc.robot.Subsystems.ShooterRollerSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;


public class RobotContainer {

  // Subsystems
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeArmSubsystem m_intakeArmSubsystem = new IntakeArmSubsystem();
  private final IntakeRollerSubsystem m_intakeRollerSubsystem = new IntakeRollerSubsystem();
  private final ShooterRollerSubsystem m_shooterRollerSubsystem = new ShooterRollerSubsystem();


  // Operator Controllers
  private final CommandXboxController m_OperatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_DriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);


  public RobotContainer() {


    DriverStation.silenceJoystickConnectionWarning(true); // Just gets rid of those annoying "joystick not connected" even though it is. Always double check though. Don't worry about it

    //Default Commands
    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.set(0)); // If no other inputs are read, motors stop (or rather, motors use 0% of power)
    m_intakeArmSubsystem.setDefaultCommand(m_intakeArmSubsystem.setAngle(Degrees.of(0)));
    m_intakeRollerSubsystem.setDefaultCommand(m_intakeRollerSubsystem.feed(0));
    configureBindings();




  }

  private void configureBindings() {

    // SHOOTER - Sets shooter speed at various RPM's
    m_OperatorController.button(1).whileTrue(m_shooterSubsystem.setVelocity(RPM.of(300))); // BE CAREFUL WHEN TESTING THIS, USE LOW PID and FF values!!!!!
    m_OperatorController.button(2).whileTrue(m_shooterSubsystem.setVelocity(RPM.of(50)));

    // INTAKE ARM - Sets the intake arm at various angles
    m_OperatorController.button(3).whileTrue(m_intakeArmSubsystem.setAngle(Degrees.of(-15)));
    m_OperatorController.button(4).whileTrue(m_intakeArmSubsystem.setAngle(Degrees.of(50)));    // BE CAREFUL WHEN TESTING THIS, USE LOW PID and FF values!!!!!
    // m_OperatorController.button(5).whileTrue(m_intakeArmSubsystem.set(0.3));

    // Intake Rollers with Shooter Rollers
    m_OperatorController.button(5).whileTrue(m_intakeRollerSubsystem.feed(0.4).alongWith(m_shooterRollerSubsystem.feed(0.3)));
    m_OperatorController.button(6).whileTrue(m_intakeRollerSubsystem.feed(-0.4)); // ShooterRollerSub has negative softlimit of 0, so not included




  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
