// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.*;
import frc.robot.commands.climber.*;
import frc.robot.subsystems.*;
import java.util.ArrayList;

public class CommandClimber extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public final Climber m_climber;

  public final XboxController m_driverController;
  public final JoystickButton m_driverButtons[] = new JoystickButton[11];
  public final POVButton m_driverPOVs[] = new POVButton[4];

  public final XboxController m_operatorController;
  public final JoystickButton m_operatorButtons[] = new JoystickButton[11];
  public final POVButton m_operatorPOVs[] = new POVButton[4];

  public final ArrayList<CommandBase> m_climberSteps = new ArrayList<CommandBase>();

  private final int kX = 0;
  private final int kY = 1;

  private double _LStickVal[] = {0, 0};
  private double _RStickVal[] = {0, 0};

  private double _climbFF = 0.0;
  private int _numberOfSteps = 0;

  public CommandClimber(RobotContainer container) {
    m_climber = container.climber;

    m_driverController = container.getDriverController();
    m_operatorController = container.getOperatorController();

    m_climberSteps.add(new ClimberStartCommand(container));
    m_climberSteps.add(new ClimberExtendCommand(container, -0.025, Constants.kClimberFeedFwd));
    m_climberSteps.add(new PivotLinkAngleCommand(container, 110.0));
    m_climberSteps.add(new ClimberExtendCommand(container, 0.125));
    // m_climberSteps.add(new PivotLinkResetCommand(container, Constants.kResetFastPivotSpeed));
    m_climberSteps.add(new ClimberResetCommand(container, Constants.kResetFastClimberSpeed));
    m_climberSteps.add(new PivotLinkAngleCommand(container, 65.0));
    m_climberSteps.add(new ClimberExtendPctCommand(container, 1.025));
    m_climberSteps.add(new PivotLinkAngleCommand(container, 40));
    m_climberSteps.add(new ClimberExtendCommand(container, -0.025, Constants.kClimberFeedFwd));
    m_climberSteps.add(new ClimberExtendCommand(container, 0.4, Constants.kClimberFeedFwd));
    m_climberSteps.add(new PivotLinkAngleCommand(container, 65.0));
    _numberOfSteps = m_climberSteps.size() - 1;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);

    m_driverButtons[Button.kBack.value] =
        new JoystickButton(m_driverController, Button.kBack.value);
    m_driverButtons[Button.kBack.value].whenPressed(() -> m_climber.reset());

    m_driverButtons[Button.kStart.value] =
        new JoystickButton(m_driverController, Button.kStart.value);
    m_driverButtons[Button.kStart.value].whenPressed(m_climberSteps.get(0));

    m_driverButtons[Button.kA.value] = new JoystickButton(m_driverController, Button.kA.value);
    m_driverButtons[Button.kA.value].whenPressed(() -> this.nextClimberSequence());

    m_driverButtons[Button.kB.value] = new JoystickButton(m_driverController, Button.kB.value);
    m_driverButtons[Button.kB.value].whenPressed(() -> this.prevClimberSequence());

    m_driverButtons[Button.kX.value] = new JoystickButton(m_driverController, Button.kX.value);
    m_driverButtons[Button.kX.value].whenPressed(() -> m_climber.stop());

    // m_driverButtons[Button.kY.value] = new JoystickButton(m_driverController, Button.kY.value);
    // m_driverButtons[Button.kY.value].whenPressed(() -> m_climber.setClimberHeightPct(1.0, 0.0));
    // --------------------------------------------------------------
    m_operatorButtons[Button.kBack.value] =
        new JoystickButton(m_operatorController, Button.kBack.value);
    m_operatorButtons[Button.kBack.value].whenPressed(() -> m_climber.reset());

    m_operatorButtons[Button.kStart.value] =
        new JoystickButton(m_operatorController, Button.kStart.value);
    m_operatorButtons[Button.kStart.value].whenPressed(m_climberSteps.get(0));

    m_operatorButtons[Button.kLeftBumper.value] =
        new JoystickButton(m_operatorController, Button.kLeftBumper.value);
    m_operatorButtons[Button.kLeftBumper.value].whenPressed(() -> m_climber.zeroClimberPosition());

    m_operatorButtons[Button.kRightBumper.value] =
        new JoystickButton(m_operatorController, Button.kRightBumper.value);
    m_operatorButtons[Button.kRightBumper.value]
        .whenPressed(() -> _climbFF = Constants.kClimberFeedFwd)
        .whenReleased(() -> _climbFF = 0.0);

    m_operatorButtons[Button.kA.value] = new JoystickButton(m_operatorController, Button.kA.value);
    m_operatorButtons[Button.kA.value].whenPressed(() -> this.prevClimberSequence());

    m_operatorButtons[Button.kB.value] = new JoystickButton(m_operatorController, Button.kB.value);
    m_operatorButtons[Button.kB.value].whenPressed(() -> this.nextClimberSequence());

    m_operatorButtons[Button.kX.value] = new JoystickButton(m_operatorController, Button.kX.value);
    m_operatorButtons[Button.kX.value].whenPressed(() -> m_climber.stop());
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker("CommandClimber init", EventImportance.kNormal);

    _LStickVal[kX] = _LStickVal[kY] = 0;
    _RStickVal[kX] = _RStickVal[kY] = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    this.doPOV(m_driverController.getPOV());
    this.doPOV(m_operatorController.getPOV());

    _LStickVal[kX] = MathUtil.applyDeadband(m_operatorController.getLeftX(), 0.1);
    _LStickVal[kY] = -MathUtil.applyDeadband(m_operatorController.getLeftY(), 0.1);
    _RStickVal[kX] = MathUtil.applyDeadband(m_operatorController.getRightX(), 0.1);
    _RStickVal[kY] = -MathUtil.applyDeadband(m_operatorController.getRightY(), 0.1);

    if ((_LStickVal[kY] != 0.0) || (_LStickVal[kX] != 0.0)) {
      m_climber.setClimberSpeed(
          (_LStickVal[kY] - _LStickVal[kX]) * 0.4,
          (_LStickVal[kY] + _LStickVal[kX]) * 0.4,
          _climbFF);
    }
    if ((_RStickVal[kY] != 0.0) || (_RStickVal[kX] != 0.0)) {
      m_climber.setPivotLinkSpeed(
          (_RStickVal[kY] - _RStickVal[kX]) * 0.1, (_RStickVal[kY] + _RStickVal[kX]) * 0.1);
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker("CommandClimber Interrupted!", EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker("CommandClimber end", EventImportance.kNormal);
  }

  public void doPOV(int pov) {
    if (pov == 0) m_climber.setClimberHeightInc(0.002, _climbFF);
    else if (pov == 90) m_climber.setPivotLinkAngleInc(-0.2);
    else if (pov == 180) m_climber.setClimberHeightInc(-0.002, _climbFF);
    else if (pov == 270) m_climber.setPivotLinkAngleInc(0.2);
  }

  public void nextClimberSequence() {
    this.doClimbingSequence(m_climber.nextClimbingSequence());
  }

  public void prevClimberSequence() {
    m_climber.prevClimbingSequence();
  }

  public void doClimbingSequence(int seq) {
    if (seq >= 0) {
      if (seq > _numberOfSteps) seq = ((seq - 1) % _numberOfSteps) + 1; // wrap around to step 1
      m_climberSteps.get(seq).schedule();
    }
  }
}
