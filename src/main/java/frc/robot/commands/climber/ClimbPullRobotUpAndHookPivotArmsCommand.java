// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbPullRobotUpAndHookPivotArmsCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbPullRobotUpAndHookPivotArmsCommand(RobotContainer container) {
    this.setName("Pull Robot Up And Hook Pivot Arms");
    addCommands(
        new ClimbPullRobotUpCommand(container), new ClimbPivotArmsOverBarCommand(container));
  }
}
