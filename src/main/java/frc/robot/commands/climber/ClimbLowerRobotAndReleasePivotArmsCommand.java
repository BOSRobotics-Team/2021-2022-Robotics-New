// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbLowerRobotAndReleasePivotArmsCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbLowerRobotAndReleasePivotArmsCommand(RobotContainer container) {
    this.setName("Lower Robot And Release Pivot Arms");
    addCommands(
        new PivotLinkAnglePctCommand(container, 0.0).withName("Pivot Arms Fully Back"),
        new ClimberExtendPctCommand(container, 0.727, Constants.kClimberFeedFwd)
            .withName("Lower Robot 6 inches"));
  }
}
