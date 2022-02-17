// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.*;
import frc.robot.subsystems.*;

public class CommandClimber extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public final Climber m_climber;

  public final XboxController m_driverController;
  public final JoystickButton m_driverButtons[] = new JoystickButton[11];
  public final POVButton m_driverPOVs[] = new POVButton[4];

  public final XboxController m_operatorController;
  public final JoystickButton m_operatorButtons[] = new JoystickButton[11];
  public final POVButton m_operatorPOVs[] = new POVButton[4];

  public final ClimberExtendCommand m_climberExtendCommand;
  public final ClimberRetrackCommand m_climberRetractCommand;
  public final ClimberResetCommand m_climberResetCommand;
  public final PivotLinkExtendCommand m_pivotLinkExtendCommand;
  public final PivotLinkRetractCommand m_pivotLinkRetractCommand;
  public final AutoClimberCommand m_autoClimberCommand;
  public final AutoClimberCommand m_autoClimberInitCommand;

  private double _leftTriggerVal = 0;
  private double _rightTriggerVal = 0;
  private boolean _manualClimber = false;

  private double _currLStickVal = 0;
  private double _currRStickVal = 0;
  private double _lastLStickVal = 0;
  private double _lastRStickVal = 0;

  public CommandClimber(RobotContainer container) {
    m_climber = container.climber;
    m_driverController = container.getDriverController();
    m_operatorController = container.getOperatorController();

    m_climberExtendCommand = new ClimberExtendCommand(container, 0.8);
    m_climberRetractCommand = new ClimberRetrackCommand(container);
    m_climberResetCommand = new ClimberResetCommand(container);
    m_pivotLinkExtendCommand = new PivotLinkExtendCommand(container, 0.25);
    m_pivotLinkRetractCommand = new PivotLinkRetractCommand(container);
    m_autoClimberCommand = new AutoClimberCommand(container);
    m_autoClimberInitCommand = new AutoClimberCommand(container, true);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);

    m_driverButtons[Button.kX.value] = new JoystickButton(m_driverController, Button.kX.value);
    m_driverButtons[Button.kX.value].whenPressed(m_autoClimberCommand);

    m_driverButtons[Button.kStart.value] =
        new JoystickButton(m_driverController, Button.kStart.value);
    m_driverButtons[Button.kStart.value].whenPressed(m_autoClimberInitCommand);

    m_driverButtons[Button.kBack.value] =
        new JoystickButton(m_driverController, Button.kBack.value);
    m_driverButtons[Button.kBack.value].whenPressed(m_climberResetCommand);

    m_driverPOVs[0] = new POVButton(m_driverController, 0);
    m_driverPOVs[0].whenPressed(m_climberExtendCommand);

    m_driverPOVs[1] = new POVButton(m_driverController, 90);
    m_driverPOVs[1].whenPressed(m_climberRetractCommand);

    m_driverPOVs[2] = new POVButton(m_driverController, 180);
    m_driverPOVs[2].whenPressed(m_pivotLinkExtendCommand);

    m_driverPOVs[3] = new POVButton(m_driverController, 270);
    m_driverPOVs[3].whenPressed(m_pivotLinkRetractCommand);

    m_operatorButtons[Button.kBack.value] =
        new JoystickButton(m_operatorController, Button.kBack.value);
    m_operatorButtons[Button.kBack.value].whenPressed(() -> m_climber.resetClimber());

    m_operatorButtons[Button.kLeftBumper.value] =
        new JoystickButton(m_operatorController, Button.kLeftBumper.value);
    m_operatorButtons[Button.kLeftBumper.value]
        .whenPressed(() -> m_climber.tripClimberRevLimitSwitches_test(true))
        .whenReleased(() -> m_climber.tripClimberRevLimitSwitches_test(false));

    m_operatorButtons[Button.kRightBumper.value] =
        new JoystickButton(m_operatorController, Button.kRightBumper.value);
    m_operatorButtons[Button.kRightBumper.value]
        .whenPressed(() -> m_climber.tripPivotRevLimitSwitches_test(true))
        .whenReleased(() -> m_climber.tripPivotRevLimitSwitches_test(false));
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("CommandClimber - initialize");
    Shuffleboard.addEventMarker(
        "CommandClimber init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    _leftTriggerVal = _rightTriggerVal = 0;
    _currLStickVal = _currRStickVal = _lastLStickVal = _lastRStickVal = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    _leftTriggerVal = m_driverController.getLeftTriggerAxis();
    _rightTriggerVal = m_driverController.getRightTriggerAxis();
    if (!_manualClimber && ((_leftTriggerVal != 0.0) || (_rightTriggerVal != 0.0)))
      _manualClimber = true;

    if (_manualClimber) {
      m_climber.setClimberHeightPct(_leftTriggerVal);
      m_climber.setPivotLinkPct(_rightTriggerVal);
      _manualClimber = ((_leftTriggerVal > 0.0) || (_rightTriggerVal > 0.0));
    }

    _currLStickVal = -m_operatorController.getLeftY();
    if (_currLStickVal != 0) {
      m_climber.setClimberSpeed(_currLStickVal);
    } else if (_lastLStickVal != 0) {
      m_climber.setClimberSpeed(0);
    }
    _lastLStickVal = _currLStickVal;

    _currRStickVal = -m_operatorController.getRightY();
    if (_currRStickVal != 0) {
      m_climber.setClimberSpeed(_currRStickVal);
    } else if (_lastRStickVal != 0) {
      m_climber.setClimberSpeed(0);
    }
    _lastRStickVal = _currRStickVal;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // System.out.println("CommandClimber - end : interrupted = " + interrupted);
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "CommandClimber Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "CommandClimber end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }
}
