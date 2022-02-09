// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.*;
import frc.robot.subsystems.*;

public class CommandClimber extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public final Climber m_climber;

  public final XboxController m_controller;
  public final JoystickButton m_buttons[] = new JoystickButton[11];

  public final ClimberExtendCommand m_climberExtendCommand;
  public final ClimberRetrackCommand m_climberRetractCommand;
  public final ClimberResetCommand m_climberResetCommand;
  public final PivotLinkExtendCommand m_pivotLinkExtendCommand;
  public final PivotLinkRetractCommand m_pivotLinkRetractCommand;
  public final AutoClimberCommand m_autoClimberCommand;

  private int _lastPOV = -1;
  private boolean _manualClimber = false;

  public CommandClimber(RobotContainer container) {
    m_climber = container.climber;
    m_controller = container.getDriverController();

    m_climberExtendCommand = new ClimberExtendCommand(container, 0.8);
    m_climberRetractCommand = new ClimberRetrackCommand(container);
    m_climberResetCommand = new ClimberResetCommand(container);
    m_pivotLinkExtendCommand = new PivotLinkExtendCommand(container, 0.25);
    m_pivotLinkRetractCommand = new PivotLinkRetractCommand(container);
    m_autoClimberCommand = new AutoClimberCommand(container);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);

    m_buttons[Button.kStart.value] = new JoystickButton(m_controller, Button.kStart.value);
    m_buttons[Button.kStart.value].whenPressed(m_autoClimberCommand);

    m_buttons[Button.kBack.value] = new JoystickButton(m_controller, Button.kBack.value);
    m_buttons[Button.kBack.value].whenPressed(m_climberResetCommand);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("CommandClimber - initialize");
    Shuffleboard.addEventMarker(
        "CommandClimber init.", this.getClass().getSimpleName(), EventImportance.kNormal);

    _lastPOV = -1;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
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
      _lastPOV = pov;
    }

    double leftTrig = m_controller.getLeftTriggerAxis();
    double rightTrig = m_controller.getRightTriggerAxis();
    if ((leftTrig != 0.0) || (rightTrig != 0.0)) _manualClimber = true;

    if (_manualClimber) {
      m_climber.setClimberHeight(leftTrig);
      m_climber.setPivotLinkDistance(rightTrig);
      if ((leftTrig == 0.0) && (rightTrig == 0.0)) _manualClimber = false;
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("CommandClimber - end : interrupted = " + interrupted);
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "CommandClimber Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "CommandClimber end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }
}
