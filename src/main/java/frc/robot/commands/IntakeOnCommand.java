// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeOnCommand extends InstantCommand {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;

    public IntakeOnCommand(Intake intake) {
        m_intake = intake;

        addRequirements(m_intake);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("IntakeOnCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_intake.runIntake(Constants.kIntakeSpeed);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("IntakeOnCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("IntakeOnCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}
