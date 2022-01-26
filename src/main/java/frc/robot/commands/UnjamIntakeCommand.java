// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class UnjamIntakeCommand extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private boolean _isFinished = false;
    private int _counter = 100;

    public UnjamIntakeCommand(Intake intake) {
        m_intake = intake;

        addRequirements(m_intake);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_intake.runIntake(Constants.kUnjamIntakeSpeed);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        _isFinished = (_counter-- < 0);
    }

     // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return _isFinished;
    }
}
