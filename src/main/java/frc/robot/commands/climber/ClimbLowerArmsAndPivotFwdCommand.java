// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbLowerArmsAndPivotFwdCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbLowerArmsAndPivotFwdCommand(RobotContainer container) {
    this.setName("Lower Climber Arms and Pivot Forward");
    addCommands(
        new ClimberResetCommand(container, Constants.kResetFastClimberSpeed)
            .withName("Reset Climbers"),
        new PivotLinkAngleCommand(container, 60.0).withName("Tilt Robot Forward (60deg)"));
  }
}
