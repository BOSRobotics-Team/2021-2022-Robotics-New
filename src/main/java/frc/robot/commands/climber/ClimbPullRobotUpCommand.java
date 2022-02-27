// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbPullRobotUpCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbPullRobotUpCommand(RobotContainer container) {
    this.setName("Pull Robot Up");
    addCommands(
        new ClimberExtendCommand(container, -0.025, Constants.kClimberFeedFwd)
            .withName("Climber Arms Fully Retracted"));
  }
}
