// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class ClimberStartCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public ClimberStartCommand(RobotContainer container) {
    addCommands(
        // new LEDAnimationCommand(container, AnimationTypes.Strobe, led1),
        new PrintCommand("Climber Extend and Tilt initial position"),
        new ClimberResetAllCommand(container),
        new ClimberExtendPctTiltPctCommand(container, 1.0, 0.25),
        new PrintCommand("Climber Extend and Tilt initial - complete"));
  }
}
