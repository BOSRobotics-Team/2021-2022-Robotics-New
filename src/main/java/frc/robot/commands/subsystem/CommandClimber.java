// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.*;
import frc.robot.commands.climber.*;
import frc.robot.subsystems.*;
import java.util.ArrayList;

public class CommandClimber extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public final Climber m_climber;

  public final LEDLights m_ledLights;

  public final XboxController m_driverController;
  public final JoystickButton m_driverButtons[] = new JoystickButton[11];
  public final POVButton m_driverPOVs[] = new POVButton[4];

  public final XboxController m_operatorController;
  public final JoystickButton m_operatorButtons[] = new JoystickButton[11];
  public final POVButton m_operatorPOVs[] = new POVButton[4];

  public final ArrayList<CommandBase> m_climberSteps = new ArrayList<CommandBase>();

  private final int kX = 0;
  private final int kY = 1;
  private final int kCurr = 0;
  private final int kLast = 1;

  private double _leftTriggerVal[] = {0, 0};
  private double _rightTriggerVal[] = {0, 0};

  private double _LStickVal[] = {0, 0};
  private double _RStickVal[] = {0, 0};

  private double _climbFF = 0.0;

  public enum ClimbingMode {
    DefaultMode,
    Manual,
    Trigger
  }

  public ClimbingMode climbingMode = ClimbingMode.DefaultMode;
  public int m_Sequence = -1;

  public CommandClimber(RobotContainer container) {
    m_climber = container.climber;
    m_ledLights = container.getLEDLights();
    m_driverController = container.getDriverController();
    m_operatorController = container.getOperatorController();

    m_climberSteps.add(new ClimberStartCommand(container));
    m_climberSteps.add(new ClimberExtendCommand(container, -0.025, Constants.kClimberFeedFwd));
    m_climberSteps.add(new PivotLinkAngleCommand(container, 110.0));
    m_climberSteps.add(new ClimberExtendCommand(container, 0.125));
    m_climberSteps.add(new PivotLinkResetCommand(container, Constants.kResetPivotSpeed * 2));
    m_climberSteps.add(new ClimberResetCommand(container, Constants.kResetClimberSpeed * 2));
    m_climberSteps.add(new PivotLinkAngleCommand(container, 75.0));
    m_climberSteps.add(new ClimberExtendPctCommand(container, 1.025));
    m_climberSteps.add(new PivotLinkAngleCommand(container, 40));
    m_climberSteps.add(new ClimberExtendCommand(container, -0.025, Constants.kClimberFeedFwd));
    m_climberSteps.add(new ClimberExtendCommand(container, 0.4, Constants.kClimberFeedFwd));
    m_climberSteps.add(new PivotLinkAngleCommand(container, 75.0));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);

    // m_driverButtons[Button.kX.value] = new JoystickButton(m_driverController, Button.kX.value);
    // m_driverButtons[Button.kX.value].whenPressed(m_autoClimberCommand);

    m_driverButtons[Button.kStart.value] =
        new JoystickButton(m_driverController, Button.kStart.value);
    m_driverButtons[Button.kStart.value].whenPressed(() -> m_climber.reset());

    // m_driverButtons[Button.kBack.value] =
    //     new JoystickButton(m_driverController, Button.kBack.value);
    // m_driverButtons[Button.kBack.value].whenPressed(m_climberResetCommand);

    // m_driverPOVs[0] = new POVButton(m_driverController, 0);
    // m_driverPOVs[0].whenPressed(m_climberExtendCommand);

    // m_driverPOVs[1] = new POVButton(m_driverController, 90);
    // m_driverPOVs[1].whenPressed(m_climberRetractCommand);

    // m_driverPOVs[2] = new POVButton(m_driverController, 180);
    // m_driverPOVs[2].whenPressed(m_pivotLinkExtendCommand);

    // m_driverPOVs[3] = new POVButton(m_driverController, 270);
    // m_driverPOVs[3].whenPressed(m_pivotLinkRetractCommand);

    m_operatorButtons[Button.kBack.value] =
        new JoystickButton(m_operatorController, Button.kBack.value);
    m_operatorButtons[Button.kBack.value].whenPressed(() -> m_climber.zeroClimberPosition());

    m_operatorButtons[Button.kStart.value] =
        new JoystickButton(m_operatorController, Button.kStart.value);
    m_operatorButtons[Button.kStart.value].whenPressed(m_climberSteps.get(0));

    m_operatorButtons[Button.kLeftBumper.value] =
        new JoystickButton(m_operatorController, Button.kLeftBumper.value);
    m_operatorButtons[Button.kLeftBumper.value].whenPressed(() -> this.prevClimberSequence());

    m_operatorButtons[Button.kRightBumper.value] =
        new JoystickButton(m_operatorController, Button.kRightBumper.value);
    m_operatorButtons[Button.kRightBumper.value].whenPressed(() -> this.nextClimberSequence());

    m_operatorButtons[Button.kLeftStick.value] =
        new JoystickButton(m_operatorController, Button.kLeftStick.value);
    m_operatorButtons[Button.kLeftStick.value].whenPressed(() -> toggleClimbingMode());

    m_operatorButtons[Button.kRightStick.value] =
        new JoystickButton(m_operatorController, Button.kRightStick.value);
    m_operatorButtons[Button.kRightStick.value]
        .whenPressed(() -> _climbFF = Constants.kClimberFeedFwd)
        .whenReleased(() -> _climbFF = 0.0);

    m_operatorButtons[Button.kY.value] = new JoystickButton(m_operatorController, Button.kY.value);
    m_operatorButtons[Button.kY.value].whenPressed(() -> m_climber.setClimberHeightPct(1.0, 0.0));

    m_operatorButtons[Button.kA.value] = new JoystickButton(m_operatorController, Button.kA.value);
    m_operatorButtons[Button.kA.value].whenPressed(
        () -> m_climber.setClimberHeight(-0.025, Constants.kClimberFeedFwd));

    m_operatorButtons[Button.kB.value] = new JoystickButton(m_operatorController, Button.kB.value);
    m_operatorButtons[Button.kB.value].whenPressed(() -> m_climber.setPivotLinkAngle(95.0));

    m_operatorButtons[Button.kX.value] = new JoystickButton(m_operatorController, Button.kX.value);
    m_operatorButtons[Button.kX.value].whenPressed(() -> m_climber.setPivotLinkAngle(70.0));
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "CommandClimber init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    _leftTriggerVal[kLast] = _leftTriggerVal[kCurr] = 0;
    _rightTriggerVal[kLast] = _rightTriggerVal[kCurr] = 0;

    _LStickVal[kX] = 0;
    _RStickVal[kX] = 0;
    _LStickVal[kY] = 0;
    _RStickVal[kY] = 0;

    SmartDashboard.putString("ClimbingMode", climbingMode.toString());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    switch (climbingMode) {
      case DefaultMode:
        {
          int pov = m_operatorController.getPOV();
          if (pov == 0) m_climber.setClimberHeightInc(0.002, _climbFF);
          else if (pov == 90) m_climber.setPivotLinkAngleInc(-0.2);
          else if (pov == 180) m_climber.setClimberHeightInc(-0.002, _climbFF);
          else if (pov == 270) m_climber.setPivotLinkAngleInc(0.2);

          // _LStickVal[kX] = MathUtil.applyDeadband(m_operatorController.getLeftX(), 0.1);
          _LStickVal[kY] = -MathUtil.applyDeadband(m_operatorController.getLeftY(), 0.1);
          // _RStickVal[kX] = MathUtil.applyDeadband(m_operatorController.getRightX(), 0.1);
          _RStickVal[kY] = -MathUtil.applyDeadband(m_operatorController.getRightY(), 0.1);

          if (_LStickVal[kY] != 0.0)
            m_climber.setClimberHeightInc(_LStickVal[kY] * 0.004, _climbFF);
          if (_RStickVal[kY] != 0.0) m_climber.setPivotLinkAngleInc(_RStickVal[kY] * 0.4);
          break;
        }
      case Manual:
        {
          _LStickVal[kY] = -m_operatorController.getLeftY() * 0.4;
          _RStickVal[kY] = -m_operatorController.getRightY() * 0.4;
          m_climber.setClimberSpeed(_LStickVal[kY], _RStickVal[kY], _climbFF);

          _LStickVal[kX] = m_operatorController.getLeftX() * 0.1;
          _RStickVal[kX] = m_operatorController.getRightX() * 0.1;
          m_climber.setPivotLinkSpeed(_LStickVal[kX], _RStickVal[kX]);
        }
      case Trigger:
        {
          _leftTriggerVal[kCurr] = m_operatorController.getLeftTriggerAxis();
          _rightTriggerVal[kCurr] = m_operatorController.getRightTriggerAxis();
          if ((_leftTriggerVal[kCurr] != 0.0)
              || (_rightTriggerVal[kCurr] != 0.0)
              || (_leftTriggerVal[kLast] != 0.0)
              || (_rightTriggerVal[kLast] != 0.0)) {
            m_climber.setClimberHeightPct(_leftTriggerVal[kCurr], _climbFF);
            m_climber.setPivotLinkAnglePct(_rightTriggerVal[kCurr]);
          }
          _leftTriggerVal[kLast] = _leftTriggerVal[kCurr];
          _rightTriggerVal[kLast] = _rightTriggerVal[kCurr];
        }
    }
  }
  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "CommandClimber Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "CommandClimber end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  public void toggleClimbingMode() {
    switch (climbingMode) {
      case DefaultMode:
        climbingMode = ClimbingMode.Manual;
        break;
      case Manual:
        climbingMode = ClimbingMode.DefaultMode;
        // climbingMode = ClimbingMode.Trigger;
        break;
      case Trigger:
        climbingMode = ClimbingMode.DefaultMode;
        break;
    }
    SmartDashboard.putString("ClimbingMode", climbingMode.toString());
  }

  public void nextClimberSequence() {
    this.doClimbingSequence(m_climber.nextClimbingSequence());
  }

  public void prevClimberSequence() {
    this.doClimbingSequence(m_climber.prevClimbingSequence());
  }

  public void doClimbingSequence(int seq) {
    if (seq >= 0) {
      m_ledLights.runLights(0, 0, 0, 0, 0, 8);
      m_ledLights.runLights(255, 255, 0, 255, 0, (seq % 8) + 1);

      if (seq > 11) {
        seq = (seq % 12) + 1; // wrap around to step 1
      }
      m_climberSteps.get(seq).schedule();
    }
  }
}
