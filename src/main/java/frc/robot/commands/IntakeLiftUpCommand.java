// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class IntakeLiftUpCommand extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;

    public IntakeLiftUpCommand(Intake intake) {
        m_intake = intake;

        addRequirements(m_intake);
        Preferences.initDouble("LiftHeight1", 1.0);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        double height = Preferences.getDouble("LiftHeight1", 1.0);

        m_intake.runLift(height);
        System.out.println("Intakelift - height = " + height);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (!m_intake.isLifting()) {
            System.out.println("Intakelift - isFinished");
            return true;
        }
        return false;
    }

}
