// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.AutoCommands.AutoNamedCommands;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.StopBits;

import static edu.wpi.first.units.Units.RPM;
// import frc.robot.subsystems.IntakeArmSubsystem;
// import frc.robot.subsystems.IntakeRollerSubsystem;
// import frc.robot.subsystems.ShooterRollerSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IndexRollerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.math.MathUtil;

// Pathplanner autobuilder imports
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotContainer {

  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final IndexRollerSubsystem m_indexRollerSubsystem = new IndexRollerSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final SendableChooser<Command> autoChooser;



  // Getter Method for robotdrive
  public DriveSubsystem getRobotDrive() {
      return m_robotDrive;
    }

  //Dont use
  // private final IntakeArmSubsystem m_intakeArmSubsystem = new IntakeArmSubsystem();
  // private final IntakeRollerSubsystem m_intakeRollerSubsystem = new IntakeRollerSubsystem();
  // private final ShooterRollerSubsystem m_shooterRollerSubsystem = new ShooterRollerSubsystem();


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
    m_climbSubsystem.setDefaultCommand(m_climbSubsystem.set(0));
    m_indexRollerSubsystem.setDefaultCommand(m_indexRollerSubsystem.set(0));
   new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem);
    // m_intakeArmSubsystem.setDefaultCommand(m_intakeArmSubsystem.setAngle(Degrees.of(0)));
    // m_intakeRollerSubsystem.setDefaultCommand(m_intakeRollerSubsystem.set(0));

    AutoNamedCommands.register(m_robotDrive, m_shooterSubsystem, m_indexRollerSubsystem);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();




  }

  private void configureBindings() {

   
    /*
     * Keep in mind, to utitlize the shooters full rpm you must allow it to reach the desired RPM.
     * This is also true for the Indexer but it's not as slow as the shooter.
     * Press R2() and allow the shooter to reach its RPM setpoint (takes about a second), 
     * then use the Indexer button to cycle full up the shooter.
     * 
     * Also, because there isn't enough physical tolerance between the Indexer and plate inside the hopper, balls
     * will somewhat struggle to pass thorugh especially if there are more than one. One fix is to use
     * smaller wheels on the indexer, and removing one wheel. This WILL allow for a much smoother
     * cycle since the ball doesn't have to fight the indexer so much.
     * 
     * If changing the wheels isn't an option, than you must simply bare with it.
     * Unfortunately, there is nothing we can do on the programming side.
     *  - Motors are controlled via dutycycle, reducing dutcycle reduces torque and speed so that won't help.
     * 
     * General tip: When using the Indexer to cycle fuel up the shooter, balls will get stuck.
     * Simply dutycycle the Indexer in the CW or CCW direction a few times and balls will go through up to the shooter.
     * 
     * Once you understand this, delete the above comments if you'd like. 
     */

    // Shoots the fuel - You can copy and paste the code below to set different buttons at different rpms. (Distances the shooter can shoot)
    m_OperatorController.R2()
                        .whileTrue(m_shooterSubsystem.setVelocity(RPM.of(-3500))); // Double check whether pos or neg direction shoots.
                                                      // .alongWith(m_indexRollerSubsystem.set(-0.9)));

    // Reverse the direction of the shooter - This WILL send balls back down and will most likely get stuck! 
    m_OperatorController.L2()
                        .whileTrue(m_shooterSubsystem.setVelocity(RPM.of(100)));
                                                      // .alongWith(m_indexRollerSubsystem.set(0.9)));


    // Test whether the positive or negative dutycycle rotates the index CW or CCW
    m_OperatorController.button(1).whileTrue(m_indexRollerSubsystem.set(0.8)); // Positive Intake
    m_OperatorController.button(2).whileTrue(m_indexRollerSubsystem.set(-0.8));         // Negative reverse


      // should run intake and indexer together!
    m_OperatorController.R1().whileTrue(
    new RunCommand(
        () -> {  m_intakeSubsystem.runVelocity(-2000);
            m_indexRollerSubsystem.set(0.9);
        },
        m_intakeSubsystem,
        m_indexRollerSubsystem
    ));
                                                      
    // When using the climb, if for any reason the robot isn't strong enough, check IRL mechanism. Or increase dutycycle but not over 1
    // Dutycycle of 0.8 is recommended.
    // Climber - Climb Up/ Climb Down -> DRIVER CONTROLLER
    m_driverController.button(1).whileTrue(m_climbSubsystem.set(0.8));
    m_driverController.button(2).whileTrue(m_climbSubsystem.set(-0.8));

    m_driverController.button(3)
            .whileTrue(
              new RunCommand(()-> m_robotDrive.setX(), m_robotDrive));

    m_driverController.button(4)
            .onTrue(
              new InstantCommand(()-> m_robotDrive.zeroHeading(), m_robotDrive));

    // ----------------------------------------------------------------------

    /*
     * Aiming and Ranging Simultaneously
     * Keep in mind this command will only work if the apriltag is in view.
     */
    // Limelight button - While Button Pressed -> Call drive command so that x and y are user controlled but rotation is computed
    m_driverController.L1().whileTrue(
      new RunCommand(
        ()-> m_robotDrive.drive(
          // If you want the robot to move to a certain distance between the apriltag and itself replace "-m_driverController.getLeftY()" with "LimelightHelpers.getTY("limelight") * -.01"
          -m_driverController.getLeftY(),
          -m_driverController.getLeftX(),
          LimelightHelpers.getTX("limelight") * -0.05,
          false
        ),
        m_robotDrive
      ));
      /* While L1 is pressed, and while the limelight detects an apriltag, the robot (limelight) will rotate itself such that it aligns with the apriltag.
       * If you see that the robot is overshooting it's alignment with the apriltag via rotation, lower or raise "-0.05" in increments of 0.01
      */


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // Or if you wish to select one specific auto, uncomment the line directly below this one. THE STRING MUST BE EXACT TO AUTO NAME IN PATHPLANNER
    // return AutoBuilder.buildAuto("Example Auto");
    // Then comment out autoChooser.getSelected()
  }
    //creates command to shoot at any one of the hub tags - print out apriltags and test!!
private Command createShootAtAnyTagCommand(int[] validTagIDs) {
    return new RunCommand(
        () -> {
            // Loop over the valid tags
            boolean found = false;
            for (int tagID : validTagIDs) {
                if (m_limelightSubsystem.hasTarget() && m_limelightSubsystem.getTagID() == tagID) {
                    double distance = m_limelightSubsystem.getSmoothedDistance();
                    double rpm = m_limelightSubsystem.getShooterRPM(distance);
                    m_shooterSubsystem.setVelocitySetpoint(RPM.of(rpm));
                    found = true;
                    break; // Stop after finding the first visible tag
                }
            }
            if (!found) {
                m_shooterSubsystem.setVelocitySetpoint(RPM.of(0)); // Stop if no tags are visible
            }
        },
        m_shooterSubsystem
    );}}
