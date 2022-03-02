// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbPullRobotUpAndMovePivotArmsUnderneathCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbPullRobotUpAndMovePivotArmsUnderneathCommand(RobotContainer container) {
    this.setName("Pull Up and Move Pivot Arms Underneath");
    addCommands(
        new ClimbPullRobotUpCommand(container),
        new ClimbLowerRobotAndReleasePivotArmsCommand(container),
        new ClimbMovePivotArmsUnderBarCommand(container));
  }
}
