// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;

public class AutoDriveTurnRelativeCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveResetPositionCommand m_command1;

  private final DriveResetHeadingCommand m_command2;
  private final AutoDriveTurnCommand m_command3;

  public AutoDriveTurnRelativeCommand(RobotContainer container, double distance, double angle) {
    m_command1 = new DriveResetPositionCommand(container);
    m_command2 = new DriveResetHeadingCommand(container);
    m_command3 = new AutoDriveTurnCommand(container, distance, angle);

    addRequirements(container.driveTrain);

    addCommands(m_command1, m_command2, m_command3);
  }
}
