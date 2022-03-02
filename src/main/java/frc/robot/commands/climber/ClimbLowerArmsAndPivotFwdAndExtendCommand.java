// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbLowerArmsAndPivotFwdAndExtendCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbLowerArmsAndPivotFwdAndExtendCommand(RobotContainer container) {
    this.setName("Lower Climber Arms, Pivot Forward, and Fully Extend");
    addCommands(
        new ClimbLowerArmsAndPivotFwdCommand(container),
        new ClimbFullyExtendClimbersCommand(container));
  }
}
