// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbPivotArmsOverBarCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbPivotArmsOverBarCommand(RobotContainer container) {
    this.setName("Pivot Arms Over Bar");
    addCommands(new PivotLinkAngleCommand(container, 97.0).withName("Pivot Arms to 97 degrees"));
  }
}
