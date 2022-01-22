// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoClimberCommand extends SequentialCommandGroup {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;

    public AutoClimberCommand(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);

        addCommands(
            new ExtendClimberCommand(climber),
            new ExtendPivotArmCommand(climber),
            new RetrackClimberCommand(climber)
        );
    }

}
