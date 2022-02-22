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
import frc.robot.commands.drivetrain.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.DriveMode;

public class CommandDriveTrain extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public final DriveTrain m_driveTrain;

  public final XboxController m_controller;
  public final JoystickButton m_buttons[] = new JoystickButton[11];

  private double _scaling = 0.2;

  public final AutoDriveStraightCommand m_autoCommand1;
  public final AutoDriveStraightRelativeCommand m_autoCommand2;
  public final AutoDriveTurnCommand m_autoCommand3;
  public final AutoDriveTurnRelativeCommand m_autoCommand4;

  public CommandDriveTrain(RobotContainer container) {
    m_driveTrain = container.driveTrain;
    m_controller = container.getDriverController();

    m_autoCommand1 = new AutoDriveStraightCommand(container, 10.0);
    m_autoCommand2 = new AutoDriveStraightRelativeCommand(container, 5.0);
    m_autoCommand3 = new AutoDriveTurnCommand(container, 5.0, 90.0);
    m_autoCommand4 = new AutoDriveTurnRelativeCommand(container, 10.0, -90.0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);

    // m_buttons[Button.kA.value] = new JoystickButton(m_controller, Button.kA.value);
    // m_buttons[Button.kB.value] = new JoystickButton(m_controller, Button.kB.value);
    // m_buttons[Button.kX.value] = new JoystickButton(m_controller, Button.kX.value);
    // m_buttons[Button.kY.value] = new JoystickButton(m_controller, Button.kY.value);
    // m_buttons[Button.kA.value].whenPressed(m_autoCommand1);
    // m_buttons[Button.kB.value].whenPressed(m_autoCommand2);
    // m_buttons[Button.kX.value].whenPressed(m_autoCommand3);
    // m_buttons[Button.kY.value].whenPressed(m_autoCommand4);

    m_buttons[Button.kLeftBumper.value] =
        new JoystickButton(m_controller, Button.kLeftBumper.value);
    m_buttons[Button.kLeftBumper.value].whenPressed(() -> m_driveTrain.addDriveScaling(-0.1));

    m_buttons[Button.kRightBumper.value] =
        new JoystickButton(m_controller, Button.kRightBumper.value);
    m_buttons[Button.kRightBumper.value].whenPressed(() -> m_driveTrain.addDriveScaling(0.1));

    m_buttons[Button.kLeftStick.value] = new JoystickButton(m_controller, Button.kLeftStick.value);
    m_buttons[Button.kLeftStick.value].whenPressed(() -> m_driveTrain.toggleDriveMode());
    // m_buttons[Button.kLeftStick.value].whenPressed(() ->
    // m_driveTrain.setUseSquares(!m_driveTrain.getUseSquares()));

    m_buttons[Button.kRightStick.value] =
        new JoystickButton(m_controller, Button.kRightStick.value);
    m_buttons[Button.kRightStick.value].whenPressed(
        () -> m_driveTrain.setUseDriveScaling(!m_driveTrain.getUseDriveScaling()));
    // m_buttons[Button.kRightStick.value].whenPressed(() ->
    // m_driveTrain.setQuickTurn(!m_driveTrain.getQuickTurn()));

    m_driveTrain.setDriveMode(DriveMode.ARCADE);
    m_driveTrain.setDriveScaling(_scaling);
    m_driveTrain.setUseDriveScaling(true);
    m_driveTrain.setUseSquares(true);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "CommandDriveTrain init.", this.getClass().getSimpleName(), EventImportance.kNormal);

    m_driveTrain.enableBrakes(true);
    m_driveTrain.enableDriveTrain(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_driveTrain.drive(m_controller);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.fullStop();
    m_driveTrain.enableBrakes(true);
    m_driveTrain.enableDriveTrain(false);

    if (interrupted) {
      Shuffleboard.addEventMarker(
          "CommandDriveTrain Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "CommandDriveTrain end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }
}
