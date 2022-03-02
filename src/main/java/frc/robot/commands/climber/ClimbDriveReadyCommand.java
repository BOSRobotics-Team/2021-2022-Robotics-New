// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbDriveReadyCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbDriveReadyCommand(RobotContainer container) {
    this.setName("Climber Drive Position");

    addCommands(
        new ClimbResetAllCommand(container).withName("Climber and Pivot Reset"),
        new PivotLinkAngleCommand(container, 80.0).withName("Move Pivot Arms Back"),
        new PivotLinkResetCommand(container, Constants.kResetPivotSpeed, true)
            .withName("Park Pivot Arms"));
  }
}
