// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.commands.ledlights.*;
import frc.robot.subsystems.*;

public class CommandLights extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public final LEDLights m_lights;

  public final XboxController m_controller;
  public final JoystickButton m_buttons[] = new JoystickButton[11];

  private final LEDAnimationCommand _animationOffCommand;
  private final LEDAnimationRotateCommand _animationRotateCommand;

  // private final LEDOnboardLightCommand _onboardLightCommand;
  // private final LEDOnboardLightCommand _onboardLightOffCommand;

  // private final LEDStripLightCommand _stripLightCommand;
  // private final LEDStripLightCommand _stripLightOffCommand;

  public CommandLights(RobotContainer container) {
    System.out.println("CommandLights constructor ");
    m_lights = container.lights;
    m_controller = container.getDriverController();

    _animationOffCommand = new LEDAnimationCommand(container);
    _animationRotateCommand = new LEDAnimationRotateCommand(container, true);
    // _onboardLightCommand = new LEDOnboardLightCommand(container, LEDColor.kWhite);
    // _onboardLightOffCommand = new LEDOnboardLightCommand(container, LEDColor.kOff);
    // _stripLightCommand = new LEDStripLightCommand(container, LEDColor.kWhite);
    // _stripLightOffCommand = new LEDStripLightCommand(container, LEDColoer.kOff);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_lights);

    // m_buttons[Button.kY.value] = new JoystickButton(m_controller, Button.kY.value);
    // m_buttons[Button.kY.value].whenPressed(_animationOffCommand);

    // m_buttons[Button.kX.value] = new JoystickButton(m_controller, Button.kX.value);
    // m_buttons[Button.kX.value].whenPressed(_animationRotateCommand);

    SmartDashboard.putData("Animation Off", _animationOffCommand);
    SmartDashboard.putData("Animation Rotate", _animationRotateCommand);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("CommandLights - initialize");
    Shuffleboard.addEventMarker(
        "CommandLights init.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (!m_lights.isAnimating()) {
      int lx = (int) ((m_controller.getLeftX() + 1.0) * 0x7FFFFF);

      int red = (lx >> 16) & 0xFF;
      int green = (lx >> 8) & 0xFF;
      int blue = lx & 0xFF;
      m_lights.runLights(red, green, blue);
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("CommandLights end - interrupted = " + interrupted);
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "CommandLights Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "CommandLights end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }
}
