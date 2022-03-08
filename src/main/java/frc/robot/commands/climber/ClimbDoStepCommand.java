// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimbDoStepCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final XboxController m_controller;

  public ClimbDoStepCommand(XboxController controller, Command... commands) {
    super(commands);
    m_controller = controller;

    if (commands.length > 0) this.setName(commands[0].getName());
  }

  @Override
  public void initialize() {
    super.initialize();
    m_controller.setRumble(RumbleType.kLeftRumble, 1.0);
    m_controller.setRumble(RumbleType.kRightRumble, 1.0);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
    m_controller.setRumble(RumbleType.kRightRumble, 0.0);
  }
}
