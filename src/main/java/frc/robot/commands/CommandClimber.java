// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;

public class CommandClimber extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public final Climber m_climber;
    public final XboxController m_controller;
    public final ExtendClimberCommand m_extendClimberCommand;
    public final RetrackClimberCommand m_retractClimberCommand;
    public final ResetClimberCommand m_resetClimberCommand;
    public final ExtendPivotLinkCommand m_extendPivotLinkCommand;
    public final RetrackPivotLinkCommand m_retractPivotLinkCommand;
    public final AutoClimberCommand m_AutoClimberCommand;

    private boolean _lastTriggerL = false;
    private boolean _lastTriggerR = false;
  
    public CommandClimber(Climber climber, XboxController controller) {
        m_climber = climber;
        m_controller = controller;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
        m_extendClimberCommand = new ExtendClimberCommand(climber);
        m_retractClimberCommand = new RetrackClimberCommand(climber);
        m_resetClimberCommand = new ResetClimberCommand(climber);
        m_extendPivotLinkCommand = new ExtendPivotLinkCommand(climber);
        m_retractPivotLinkCommand = new RetrackPivotLinkCommand(climber);
        m_AutoClimberCommand = new AutoClimberCommand(climber);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        _lastTriggerL = _lastTriggerR = false;

        JoystickButton a_Button = new JoystickButton(m_controller, 1);
        JoystickButton b_Button = new JoystickButton(m_controller, 2);
        JoystickButton x_Button = new JoystickButton(m_controller, 3);
        JoystickButton y_Button = new JoystickButton(m_controller, 4);
        JoystickButton left_Bumper = new JoystickButton(m_controller, 5);
        //JoystickButton right_Bumper = new JoystickButton(m_controller, 6);
        JoystickButton back_Button = new JoystickButton(m_controller, 7);
        JoystickButton start_Button = new JoystickButton(m_controller, 8);
        //JoystickButton left_Stick = new JoystickButton(m_controller, 9);
        //JoystickButton right_Stick = new JoystickButton(m_controller, 10);

        a_Button.whenPressed(m_extendClimberCommand);
        b_Button.whenPressed(m_retractClimberCommand);
    
        x_Button.whenPressed(m_extendPivotLinkCommand);
        y_Button.whenPressed(m_retractPivotLinkCommand);
    
        start_Button.whenPressed(m_resetClimberCommand);
        left_Bumper.whenPressed(m_AutoClimberCommand);
    
        back_Button.whenPressed(() -> m_climber.tripRevLimitSwitches_test(true))
                            .whenReleased(() -> m_climber.tripRevLimitSwitches_test(false));
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