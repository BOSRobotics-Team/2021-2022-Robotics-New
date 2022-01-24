// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;

public class CommandClimber extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public final Climber m_climber;
    public final XboxController m_controller;

    private boolean _lastTriggerL = false;
    private boolean _lastTriggerR = false;

    public CommandClimber(Climber climber, XboxController controller) {
        m_climber = climber;
        m_controller = controller;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        _lastTriggerL = _lastTriggerR = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_climber.setClimber(m_controller.getLeftY());
        m_climber.setPivotLink(m_controller.getRightY());

        double triggerL = m_controller.getLeftTriggerAxis();
        if ((triggerL >= 0.5) && !_lastTriggerL) { 
        }
        _lastTriggerL = (triggerL > 0.5);

        double triggerR = m_controller.getRightTriggerAxis();
        if ((triggerR >= 0.5) && !_lastTriggerR) {
        }
        _lastTriggerR = (triggerR > 0.5);

//        m_climber.logPeriodic();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
    }
}