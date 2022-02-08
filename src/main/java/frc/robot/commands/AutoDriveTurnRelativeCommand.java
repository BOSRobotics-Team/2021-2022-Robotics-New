// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoDriveTurnRelativeCommand extends SequentialCommandGroup {
@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final DriveTrain m_driveTrain;

    public AutoDriveTurnRelativeCommand(RobotContainer container, double distance, double angle) {
        m_driveTrain = container.driveTrain;

        addRequirements(m_driveTrain);

        addCommands(
            new DriveResetPositionCommand(container),
            new DriveResetHeadingCommand(container),
            new AutoDriveTurnCommand(container, distance, angle)
        );
   }
}    