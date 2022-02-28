// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbFullyExtendClimbersCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbFullyExtendClimbersCommand(RobotContainer container) {
    this.setName("Fully Extend Climbers");
    addCommands(new ClimberExtendCommand(container, 0.56).withName("Extend Climbers to MaxLength"));
  }
}
