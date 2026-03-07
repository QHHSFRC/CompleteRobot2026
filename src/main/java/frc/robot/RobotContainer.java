// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;


public class RobotContainer {

  // Subsystems
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeArmSubsystem m_intakeArmSubsystem = new IntakeArmSubsystem();
  private final IntakeRollerSubsystem m_intakeRollerSubsystem = new IntakeRollerSubsystem();
  private final ShooterRollerSubsystem m_shooterRollerSubsystem = new ShooterRollerSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();


  // Operator Controllers
  private final CommandPS4Controller m_OperatorController = new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);


  public RobotContainer() {

    // Swerve
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));


    DriverStation.silenceJoystickConnectionWarning(true); // Just gets rid of those annoying "joystick not connected" even though it is. Always double check though. Don't worry about it

    //Default Commands
    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.set(0)); // If no other inputs are read, motors stop (or rather, motors use 0% of power)
    m_intakeArmSubsystem.setDefaultCommand(m_intakeArmSubsystem.setAngle(Degrees.of(0)));
    m_intakeRollerSubsystem.setDefaultCommand(m_intakeRollerSubsystem.set(0));
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

    // Intake balls
    m_OperatorController.button(5).whileTrue(m_intakeRollerSubsystem.set(0.4));
    m_OperatorController.button(6).whileTrue(m_intakeRollerSubsystem.set(-0.4));

    // // Spin Intake Rollers (not intake) and shoot
    m_OperatorController.button(7)
      .whileTrue(m_intakeRollerSubsystem.set(0.4)
                                        .alongWith(m_shooterSubsystem.setVelocity(RPM.of(300))));



    m_driverController.button(6)
            .whileTrue(
              new RunCommand(()-> m_robotDrive.setX(), m_robotDrive));

    // new JoystickButton(m_driverController, Button.kStart.value)    // "kStart" doesn't work, probably redacted.
    //     .onTrue(new InstantCommand(
    //         () -> m_robotDrive.zeroHeading(),
    //         m_robotDrive));




  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
