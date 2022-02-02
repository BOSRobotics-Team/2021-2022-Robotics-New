// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotLinkRetractCommand extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;

    public PivotLinkRetractCommand(Climber climber) {
        m_climber = climber;

        addRequirements(m_climber);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("PivotLinkRetractCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        m_climber.runPivotLink(0.0);
        System.out.println("extendPivotLink - distance = 0.0");
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("PivotLinkRetractCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("PivotLinkRetractCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return !m_climber.isPivoting();
    }
}
