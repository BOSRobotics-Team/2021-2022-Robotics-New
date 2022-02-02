// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeLiftUpCommand extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private double m_height;

    public IntakeLiftUpCommand(Intake intake, double height) {
        m_intake = intake;
        m_height = height;

        addRequirements(m_intake);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("IntakeLiftUpCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        m_intake.runLift(m_height);
        System.out.println("Intakelift - height = " + m_height);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("IntakeLiftUpCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("IntakeLiftUpCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return !m_intake.isLifting();
    }
}
