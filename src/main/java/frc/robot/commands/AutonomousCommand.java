// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.drivetrain.*;

public class AutonomousCommand extends ParallelCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public AutonomousCommand(RobotContainer container) {
    Preferences.initDouble("AutonomousDistance1", 2.0);

    double distance1 = Preferences.getDouble("AutonomousDistance1", 2.0);
    addCommands(
        new AutoDriveStraightRelativeCommand(container, distance1),
        new ClimbResetAllCommand(container));
  }
}
