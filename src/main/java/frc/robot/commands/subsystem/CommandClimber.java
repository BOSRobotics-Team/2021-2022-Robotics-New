// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.*;
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

  private final int kX = 0;
  private final int kY = 1;
  private final int kCurr = 0;
  private final int kLast = 1;

  private double _leftTriggerVal[] = {0, 0};
  private double _rightTriggerVal[] = {0, 0};

  private double _LStickVal[][] = {{0, 0}, {0, 0}};
  private double _RStickVal[][] = {{0, 0}, {0, 0}};

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
    m_operatorPOVs[0] = new POVButton(m_operatorController, 0);
    m_operatorPOVs[0].whileHeld(
        () -> m_climber.setClimberHeightPct(m_climber.getClimberHeightPct() + 0.01));

    m_operatorPOVs[1] = new POVButton(m_operatorController, 90);
    m_operatorPOVs[1].whileHeld(
        () -> m_climber.setClimberHeightPct(m_climber.getClimberHeightPct() - 0.01));

    m_operatorPOVs[2] = new POVButton(m_operatorController, 180);
    m_operatorPOVs[2].whileHeld(
        () -> m_climber.setPivotLinkAnglePct(m_climber.getPivotLinkAnglePct() + 0.01));

    m_operatorPOVs[3] = new POVButton(m_operatorController, 270);
    m_operatorPOVs[3].whileHeld(
        () -> m_climber.setPivotLinkAnglePct(m_climber.getPivotLinkAnglePct() - 0.01));
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("CommandClimber - initialize");
    Shuffleboard.addEventMarker(
        "CommandClimber init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    _leftTriggerVal[kLast] = _leftTriggerVal[kCurr] = 0;
    _rightTriggerVal[kLast] = _rightTriggerVal[kCurr] = 0;

    _LStickVal[kX][kCurr] = _LStickVal[kX][kLast] = 0;
    _RStickVal[kX][kCurr] = _RStickVal[kX][kLast] = 0;
    _LStickVal[kY][kCurr] = _LStickVal[kY][kLast] = 0;
    _RStickVal[kY][kCurr] = _RStickVal[kY][kLast] = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    _leftTriggerVal[kCurr] = m_driverController.getLeftTriggerAxis();
    _rightTriggerVal[kCurr] = m_driverController.getRightTriggerAxis();
    if ((_leftTriggerVal[kCurr] != 0.0)
        || (_rightTriggerVal[kCurr] != 0.0)
        || (_leftTriggerVal[kLast] != 0.0)
        || (_rightTriggerVal[kLast] != 0.0)) {
      m_climber.setClimberHeightPct(_leftTriggerVal[kCurr]);
      m_climber.setPivotLinkAnglePct(_rightTriggerVal[kCurr]);
    }
    _leftTriggerVal[kLast] = _leftTriggerVal[kCurr];
    _rightTriggerVal[kLast] = _rightTriggerVal[kCurr];

    _LStickVal[kY][kCurr] = -m_operatorController.getLeftY() * 0.4;
    _RStickVal[kY][kCurr] = -m_operatorController.getRightY() * 0.4;
    if ((_LStickVal[kY][kCurr] != 0)
        || (_RStickVal[kY][kCurr] != 0)
        || (_LStickVal[kY][kLast] != 0)
        || (_RStickVal[kY][kLast] != 0)) {
      m_climber.setClimberSpeed(_LStickVal[kY][kCurr], _RStickVal[kY][kCurr]);
    }
    _LStickVal[kY][kLast] = _LStickVal[kY][kCurr];
    _RStickVal[kY][kLast] = _RStickVal[kY][kCurr];

    _LStickVal[kX][kCurr] = m_operatorController.getLeftX() * 0.1;
    _RStickVal[kX][kCurr] = m_operatorController.getRightX() * 0.1;
    if ((_LStickVal[kX][kCurr] != 0)
        || (_RStickVal[kX][kCurr] != 0)
        || (_LStickVal[kX][kLast] != 0)
        || (_RStickVal[kX][kLast] != 0)) {
      m_climber.setPivotLinkSpeed(_LStickVal[kX][kCurr], _RStickVal[kX][kCurr]);
    }
    _LStickVal[kX][kLast] = _LStickVal[kX][kCurr];
    _RStickVal[kX][kLast] = _RStickVal[kX][kCurr];
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
