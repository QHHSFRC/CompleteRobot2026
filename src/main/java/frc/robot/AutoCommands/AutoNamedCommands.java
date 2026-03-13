// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.IndexRollerSubsystem;
import frc.robot.Subsystems.DriveSubsystem;

public class AutoNamedCommands {
    private AutoNamedCommands() {}

    public static void register(DriveSubsystem drive, ShooterSubsystem shooter, IndexRollerSubsystem indexer) {
        // STOP COMMAND
        NamedCommands.registerCommand("STOP Shooter", shooter.set(0));
        NamedCommands.registerCommand("STOP Indexer", indexer.set(0));
        // Shoot (Shooter)
        NamedCommands.registerCommand("Shoot (Far)", shooter.setVelocity(RPM.of(-3500)));
        NamedCommands.registerCommand("Shoot (Medium)", shooter.setVelocity(RPM.of(-2500)));
        NamedCommands.registerCommand("Shoot (Close)", shooter.setVelocity(RPM.of(-1500)));
        NamedCommands.registerCommand("Cycle Fuel to Shooter", indexer.set(0.8)); // Indexer Feeds to shooter.
        // Intake Fuel (Indexer and shooter)
        NamedCommands.registerCommand("Shooter Intake Fuel", shooter.setVelocity(RPM.of(1500)));
        NamedCommands.registerCommand("Indexer Intake Fuel", indexer.set(-0.8));
    }

}
