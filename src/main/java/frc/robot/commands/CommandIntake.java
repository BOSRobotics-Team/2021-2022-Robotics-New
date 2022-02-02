// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CommandIntake extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public final Intake m_intake;
    public final XboxController m_controller;
    // public final JoystickButton right_Bumper;
    // public final JoystickButton left_Stick;
    // public final JoystickButton right_Stick;

    private final IntakeOnCommand _onCommand;
    private final IntakeOffCommand _offCommand;
    // private final IntakeUnjamCommand _unjamCommand;

    private final IntakeLiftUpCommand _upCommand;
    private final IntakeLiftDownCommand _dnCommand;
    // private final IntakeLiftResetCommand _resetLiftCommand;

    private boolean _lastTriggerL = false;
    private boolean _lastTriggerR = false;

    public CommandIntake(Intake intake, XboxController controller) {
        m_intake = intake;
        m_controller = controller;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);

        // right_Bumper = new JoystickButton(m_controller, 6);
        // left_Stick = new JoystickButton(m_controller, 9);
        // right_Stick = new JoystickButton(m_controller, 10);

        _onCommand = new IntakeOnCommand(m_intake);
        _offCommand = new IntakeOffCommand(m_intake);
        // _unjamCommand = new IntakeUnjamCommand(m_intake);

        _upCommand = new IntakeLiftUpCommand(m_intake, 1.2);
        _dnCommand = new IntakeLiftDownCommand(m_intake);
        // _resetLiftCommand = new IntakeLiftResetCommand(m_intake);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("CommandIntake init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        _lastTriggerL = _lastTriggerR = false;

        // left_Stick.whenPressed(_resetLiftCommand);
        // right_Stick.whenPressed(_unjamCommand);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double triggerL = m_controller.getLeftTriggerAxis();
        if ((triggerL > 0.5) && !_lastTriggerL) { 
            _onCommand.schedule();
        } else if ((triggerL <= 0.5) && _lastTriggerL) {
            _offCommand.schedule();
        }
        _lastTriggerL = (triggerL > 0.5);

        double triggerR = m_controller.getRightTriggerAxis();
        if ((triggerR > 0.5) && !_lastTriggerR) {
            _upCommand.schedule();
        } else if ((triggerL <= 0.5) && _lastTriggerL) {
            _dnCommand.schedule();
        }
        _lastTriggerR = (triggerR > 0.5);

    //    m_intake.logPeriodic();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("CommandLights Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("CommandLights end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
    }
}