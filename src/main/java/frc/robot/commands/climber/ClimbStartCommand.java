// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbStartCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbStartCommand(RobotContainer container) {
    this.setName("Climber Start Position");

    addCommands(
        new ClimbResetAllCommand(container).withName("Climber and Pivot Reset"),
        new ClimberExtendPctTiltAngleCommand(container, 1.0, 75)
            .withName("Climber Extend and Tilt initial position"));
  }
}
