package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import static edu.wpi.first.units.Units.RPM;

public class ShootAtTagCommand {

    /**
     * Factory method to create a shoot-at-tag command.
     * @param shooter Your ShooterSubsystem
     * @param limelight Your LimelightSubsystem
     * @param targetTagID The AprilTag ID to track
     * @return A command that spins the shooter automatically based on tag distance
     */
    public static Command create(ShooterSubsystem shooter, LimelightSubsystem limelight, int targetTagID) {
        return new RunCommand(
            () -> {
                if (limelight.hasTarget() && limelight.getTagID() == targetTagID) {
                    double distance = limelight.getSmoothedDistance();
                    double rpm = limelight.getShooterRPM(distance);
                    shooter.setVelocitySetpoint(RPM.of(rpm));
                } else {
                    shooter.setVelocitySetpoint(RPM.of(0)); // stop or idle if no target
                }
            },
            shooter // requirement: prevents other commands from using the shooter
        );
    }
}