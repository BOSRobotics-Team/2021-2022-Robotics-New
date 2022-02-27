// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbMovePivotArmsUnderBarCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbMovePivotArmsUnderBarCommand(RobotContainer container) {
    this.setName("Move Pivot Arms Under Bar");
    addCommands(new PivotLinkAngleCommand(container, 65.0).withName("Rotate Pivot Arms (65 deg))"));
  }
}
