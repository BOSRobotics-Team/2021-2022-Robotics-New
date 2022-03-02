// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbHookClimbersOverBarCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbHookClimbersOverBarCommand(RobotContainer container) {
    this.setName("Hook Climbers Over Bar");
    addCommands(new PivotLinkAngleCommand(container, 43.0).withName("Pivot Arms Fully Forward"));
  }
}
