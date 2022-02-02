// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandClimber extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public final Climber m_climber;
    public final XboxController m_controller;
    public final JoystickButton a_Button;
    // public final JoystickButton b_Button;
    // public final JoystickButton x_Button;
    // public final JoystickButton y_Button;
    // public final JoystickButton left_Bumper;
    // public final JoystickButton right_Bumper;
    public final JoystickButton back_Button;
    public final JoystickButton start_Button;
    // public final JoystickButton left_Stick;
    // public final JoystickButton right_Stick;

    public final ClimberExtendCommand m_climberExtendCommand;
    public final ClimberRetrackCommand m_climberRetractCommand;
    public final ClimberResetCommand m_climberResetCommand;
    public final ClimberTripLimitSwitchesCommand m_climberTripLimitSwitchesCommand;
    public final PivotLinkExtendCommand m_pivotLinkExtendCommand;
    public final PivotLinkRetractCommand m_pivotLinkRetractCommand;
    public final AutoClimberCommand m_autoClimberCommand;

    private boolean _lastTriggerL = false;
    private boolean _lastTriggerR = false;
    private int     _lastPOV = -1;
  
    public CommandClimber(Climber climber, XboxController controller) {
        m_climber = climber;
        m_controller = controller;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);

        a_Button = new JoystickButton(m_controller, 1);
        // b_Button = new JoystickButton(m_controller, 2);
        // x_Button = new JoystickButton(m_controller, 3);
        // y_Button = new JoystickButton(m_controller, 4);
        // left_Bumper = new JoystickButton(m_controller, 5);
        // right_Bumper = new JoystickButton(m_controller, 6);

        back_Button = new JoystickButton(m_controller, 7);
        start_Button = new JoystickButton(m_controller, 8);
        // left_Stick = new JoystickButton(m_controller, 9);
        // right_Stick = new JoystickButton(m_controller, 10);

        m_climberExtendCommand = new ClimberExtendCommand(climber, 1.5);
        m_climberRetractCommand = new ClimberRetrackCommand(climber);
        m_climberResetCommand = new ClimberResetCommand(climber);
        m_climberTripLimitSwitchesCommand = new ClimberTripLimitSwitchesCommand(climber);
        m_pivotLinkExtendCommand = new PivotLinkExtendCommand(climber, 0.75);
        m_pivotLinkRetractCommand = new PivotLinkRetractCommand(climber);
        m_autoClimberCommand = new AutoClimberCommand(climber);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("CommandClimber init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        _lastTriggerL = _lastTriggerR = false;
        _lastPOV = -1;

        a_Button.whenPressed(m_autoClimberCommand);
        // b_Button.whenPressed(m_climberRetractCommand);
    
        // x_Button.whenPressed(m_pivotLinkExtendCommand);
        // y_Button.whenPressed(m_pivotLinkRetractCommand);
    
        // left_Bumper.whenPressed(m_autoClimberCommand);

        start_Button.whenPressed(m_climberResetCommand);    
        back_Button.whenPressed(m_climberTripLimitSwitchesCommand);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double triggerL = m_controller.getLeftTriggerAxis();
        if ((triggerL > 0.5) && !_lastTriggerL) { 
        }
        _lastTriggerL = (triggerL > 0.5);

        double triggerR = m_controller.getRightTriggerAxis();
        if ((triggerR > 0.5) && !_lastTriggerR) {
        }
        _lastTriggerR = (triggerR > 0.5);

        int pov = m_controller.getPOV();
        if (pov != _lastPOV) {
            if (pov == 0) { // up
                m_climberExtendCommand.schedule();
            } else if (pov == 180) { // down
                m_climberRetractCommand.schedule();
            } else if (pov == 270) { // left
                m_pivotLinkExtendCommand.schedule();
            } else if (pov == 90) { // right
                m_pivotLinkRetractCommand.schedule();
            }
        }
        _lastPOV = pov;

        m_climber.setClimber(m_controller.getLeftY() * 0.5);
        m_climber.setPivotLink(m_controller.getRightY() * 0.5);

        m_climber.logPeriodic();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("CommandClimber Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("CommandClimber end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
    }
}