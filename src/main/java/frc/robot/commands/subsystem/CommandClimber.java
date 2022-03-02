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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  public final ArrayList<CommandBase> m_newclimberSteps = new ArrayList<CommandBase>();

  private final int kX = 0;
  private final int kY = 1;

  private double _lStickVal[] = {0, 0};
  private double _rStickVal[] = {0, 0};
  private boolean _wasLStickActive = false;
  private boolean _wasRStickActive = false;

  private double _climbFF = 0.0;
  private int _numberOfSteps = 0;
  private int _numberOfNewSteps = 0;

  public CommandClimber(RobotContainer container) {
    m_climber = container.climber;

    m_driverController = container.getDriverController();
    m_operatorController = container.getOperatorController();

    m_climberSteps.add(
        new ClimbStartCommand(container).withName("Climber Extend and Tilt initial position"));
    m_climberSteps.add(
        new ClimberExtendCommand(container, -0.025, Constants.kClimberFeedFwd)
            .withName("Pull Robot Up"));
    m_climberSteps.add(new PivotLinkAngleCommand(container, 105.0).withName("Pivot Arms Over Bar"));
    m_climberSteps.add(
        new ClimberExtendCommand(container, 0.125).withName("Raise Climbers 5 inches"));
    // m_climberSteps.add(new PivotLinkResetCommand(container,
    // Constants.kResetFastPivotSpeed).withName("Reset Pivot Arms"));
    m_climberSteps.add(
        new ClimberResetCommand(container, Constants.kResetFastClimberSpeed)
            .withName("Reset Climbers"));
    m_climberSteps.add(
        new PivotLinkAngleCommand(container, 65.0).withName("Tilt Robot Forward (65deg)"));
    m_climberSteps.add(
        new ClimberExtendPctCommand(container, 1.025).withName("Fully Extend Climbers"));
    m_climberSteps.add(
        new PivotLinkAngleCommand(container, 40).withName("Tilt Robot Fully Forward"));
    m_climberSteps.add(
        new ClimberExtendCommand(container, -0.025, Constants.kClimberFeedFwd)
            .withName("Pull Robot Up"));
    m_climberSteps.add(
        new PivotLinkAngleCommand(container, 35.0).withName("Pivot Arms Fully Back"));
    m_climberSteps.add(
        new ClimberExtendCommand(container, 0.4, Constants.kClimberFeedFwd)
            .withName("Lower Robot 6 inches"));
    m_climberSteps.add(
        new PivotLinkAngleCommand(container, 65.0).withName("Move Pivot Arms Forward"));
    _numberOfSteps = m_climberSteps.size() - 1;

    m_newclimberSteps.add(new ClimbStartCommand(container));
    m_newclimberSteps.add(new ClimbPullRobotUpCommand(container));
    m_newclimberSteps.add(new ClimbPivotArmsOverBarCommand(container));
    m_newclimberSteps.add(new ClimbReleaseClimberArmsCommand(container));
    m_newclimberSteps.add(new ClimbLowerArmsAndPivotFwdCommand(container));
    m_newclimberSteps.add(new ClimbFullyExtendClimbersCommand(container));
    m_newclimberSteps.add(new ClimbHookClimbersOverBarCommand(container));
    m_newclimberSteps.add(new ClimbPullRobotUpCommand(container));
    m_newclimberSteps.add(new ClimbLowerRobotAndReleasePivotArmsCommand(container));
    m_newclimberSteps.add(new ClimbMovePivotArmsUnderBarCommand(container));
    _numberOfNewSteps = m_newclimberSteps.size() - 1;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);

    m_driverButtons[Button.kBack.value] =
        new JoystickButton(m_driverController, Button.kBack.value);
    m_driverButtons[Button.kBack.value].whenPressed(new ClimbDriveReadyCommand(container));

    m_driverButtons[Button.kStart.value] =
        new JoystickButton(m_driverController, Button.kStart.value);
    m_driverButtons[Button.kStart.value].whenPressed(() -> this.startClimbing());

    m_driverButtons[Button.kA.value] = new JoystickButton(m_driverController, Button.kA.value);
    m_driverButtons[Button.kA.value].whenPressed(() -> this.nextClimberSequence());

    m_driverButtons[Button.kB.value] = new JoystickButton(m_driverController, Button.kB.value);
    m_driverButtons[Button.kB.value].whenPressed(() -> this.prevClimberSequence());

    m_driverButtons[Button.kX.value] = new JoystickButton(m_driverController, Button.kX.value);
    m_driverButtons[Button.kX.value].whenPressed(() -> this.stop());

    m_driverButtons[Button.kY.value] = new JoystickButton(m_driverController, Button.kY.value);
    m_driverButtons[Button.kY.value].whenPressed(() -> this.nextNewClimberSequence());

    // m_driverButtons[Button.kY.value] = new JoystickButton(m_driverController, Button.kY.value);
    // m_driverButtons[Button.kY.value].whenPressed(() -> m_climber.setClimberHeightPct(1.0, 0.0));
    // --------------------------------------------------------------
    m_operatorButtons[Button.kBack.value] =
        new JoystickButton(m_operatorController, Button.kBack.value);
    m_operatorButtons[Button.kBack.value].whenPressed(() -> this.resetClimbing());

    m_operatorButtons[Button.kStart.value] =
        new JoystickButton(m_operatorController, Button.kStart.value);
    m_operatorButtons[Button.kStart.value].whenPressed(() -> this.startClimbing());

    m_operatorButtons[Button.kA.value] = new JoystickButton(m_operatorController, Button.kA.value);
    m_operatorButtons[Button.kA.value].whenPressed(() -> this.nextClimberSequence());

    m_operatorButtons[Button.kB.value] = new JoystickButton(m_operatorController, Button.kB.value);
    m_operatorButtons[Button.kB.value].whenPressed(() -> this.prevClimberSequence());

    m_operatorButtons[Button.kX.value] = new JoystickButton(m_operatorController, Button.kX.value);
    m_operatorButtons[Button.kX.value].whenPressed(() -> this.stop());

    // m_operatorButtons[Button.kLeftBumper.value] =
    //     new JoystickButton(m_operatorController, Button.kLeftBumper.value);
    // m_operatorButtons[Button.kLeftBumper.value].whenPressed(() ->
    // m_climber.zeroClimberPosition());

    // m_operatorButtons[Button.kRightBumper.value] =
    //     new JoystickButton(m_operatorController, Button.kRightBumper.value);
    // m_operatorButtons[Button.kRightBumper.value]
    //     .whenPressed(() -> _climbFF = Constants.kClimberFeedFwd)
    //     .whenReleased(() -> _climbFF = 0.0);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker("CommandClimber init", EventImportance.kNormal);

    _lStickVal[kX] = _lStickVal[kY] = 0;
    _rStickVal[kX] = _rStickVal[kY] = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    this.doPOV(m_driverController.getPOV());
    this.doPOV(m_operatorController.getPOV());

    _lStickVal[kX] = MathUtil.applyDeadband(m_operatorController.getLeftX(), 0.1);
    _lStickVal[kY] =
        MathUtil.applyDeadband(
            m_operatorController.getLeftY(), 0.1); // request is for inverted Y axis
    if ((_lStickVal[kX] != 0.0) || (_lStickVal[kY] != 0.0)) {
      _wasLStickActive = true;
      m_climber.setClimberSpeed(
          (_lStickVal[kY] - _lStickVal[kX]) * 0.3,
          (_lStickVal[kY] + _lStickVal[kX]) * 0.3,
          _climbFF);
    }
    if (_wasLStickActive && (_lStickVal[kX] == 0.0) && (_lStickVal[kY] == 0.0)) {
      m_climber.setClimberSpeed(0.0, 0.0);
      _wasLStickActive = false;
    }

    _rStickVal[kX] = MathUtil.applyDeadband(m_operatorController.getRightX(), 0.1);
    _rStickVal[kY] = -MathUtil.applyDeadband(m_operatorController.getRightY(), 0.1);
    if ((_rStickVal[kX] != 0.0) || (_rStickVal[kY] != 0.0)) {
      _wasRStickActive = true;
      m_climber.setPivotLinkSpeed(
          (_rStickVal[kX] - _rStickVal[kY]) * 0.1, (_rStickVal[kX] + _rStickVal[kY]) * 0.1);
    }
    if (_wasRStickActive && (_rStickVal[kX] == 0.0) && (_rStickVal[kY] == 0.0)) {
      m_climber.setPivotLinkSpeed(0.0, 0.0);
      _wasRStickActive = false;
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
    else if (pov == 90) m_climber.setPivotLinkAngleInc(-0.35);
    else if (pov == 180) m_climber.setClimberHeightInc(-0.002, _climbFF);
    else if (pov == 270) m_climber.setPivotLinkAngleInc(0.35);
  }

  public void resetClimbing() {
    m_climber.reset();
  }

  public void startClimbing() {
    m_climberSteps.get(0).schedule();
  }

  public int getStepFromSequence(int seq, int numSteps) {
    return (seq <= numSteps) ? seq : ((seq - 1) % numSteps) + 1; // wrap around to step 1
  }

  public void nextClimberSequence() {
    this.doClimbingSequence(m_climber.nextClimbingSequence());
  }

  public void prevClimberSequence() {
    m_climber.prevClimbingSequence();
  }

  public void stop() {
    m_climber.stop();
    CommandScheduler.getInstance().cancelAll();
  }

  public void doClimbingSequence(int seq) {
    if ((seq >= 0) && (seq <= 25)) {
      CommandBase cmd = m_climberSteps.get(getStepFromSequence(seq, _numberOfSteps));
      cmd.schedule();

      SmartDashboard.putString("Curr Sequence", cmd.getName());
      SmartDashboard.putString(
          "Next Sequence",
          m_climberSteps.get(getStepFromSequence(seq + 1, _numberOfSteps)).getName());
    }
  }

  public void nextNewClimberSequence() {
    this.doNewClimbingSequence(m_climber.nextClimbingSequence());
  }

  public void doNewClimbingSequence(int seq) {
    if ((seq >= 0) && (seq <= 21)) {
      CommandBase cmd = m_newclimberSteps.get(getStepFromSequence(seq, _numberOfNewSteps));
      cmd.schedule();

      SmartDashboard.putString("Curr Sequence", cmd.getName());
      SmartDashboard.putString(
          "Next Sequence",
          m_newclimberSteps.get(getStepFromSequence(seq + 1, _numberOfNewSteps)).getName());
    }
  }
}
