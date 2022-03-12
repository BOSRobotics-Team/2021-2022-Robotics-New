// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimbReleaseClimberArmsCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimbReleaseClimberArmsCommand(RobotContainer container) {
    this.setName("Release Climbing Arms");
    addCommands(
        new ClimberExtendPctCommand(container, 0.22).withName("Raise Climbers 5 inches")
        //  new PivotLinkResetCommand(container, Constants.kResetFastPivotSpeed).withName("Reset
        // Pivot Arms")
        );
  }
}
