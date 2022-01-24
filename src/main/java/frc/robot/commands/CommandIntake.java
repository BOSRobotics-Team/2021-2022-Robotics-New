// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;

public class CommandIntake extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public final Intake m_intake;
    public final XboxController m_controller;
    public final JoystickButton back_Button;

    private final IntakeOnCommand _onCommand;
    private final IntakeOffCommand _offCommand;
    private final UnjamIntakeCommand _unjamCommand;

    private boolean _lastTriggerL = false;
    private boolean _lastTriggerR = false;

    public CommandIntake(Intake intake, XboxController controller) {
        m_intake = intake;
        m_controller = controller;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);

        back_Button = new JoystickButton(m_controller, 7);

        _onCommand = new IntakeOnCommand(m_intake);
        _offCommand = new IntakeOffCommand(m_intake);
        _unjamCommand = new UnjamIntakeCommand(m_intake);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        _lastTriggerL = _lastTriggerR = false;

        back_Button.whenPressed(_unjamCommand);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double triggerL = m_controller.getLeftTriggerAxis();
        if ((triggerL >= 0.5) && !_lastTriggerL) { 
            _onCommand.schedule();
        } else if ((triggerL < 0.5) && _lastTriggerL) {
            _offCommand.schedule();
        }
        _lastTriggerL = (triggerL > 0.5);

        double triggerR = m_controller.getRightTriggerAxis();
        if ((triggerR >= 0.5) && !_lastTriggerR) {
        }
        _lastTriggerR = (triggerR > 0.5);

       m_intake.logPeriodic();
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