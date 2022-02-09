// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ledlights;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDLights.LEDColor;

public class LEDOnboardLightCommand extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDLights m_lights;

  private final LEDColor m_color;

  public LEDOnboardLightCommand(RobotContainer container, LEDColor color) {
    m_lights = container.lights;
    m_color = color;

    addRequirements(m_lights);
  }

  public LEDOnboardLightCommand(RobotContainer container) {
    this(container, LEDColor.kOff);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "LEDOnboardLightCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    System.out.println("LEDOnboardLightCommand init : color = " + m_color);
  }

  // Called when this Command is scheduled to run
  @Override
  public void execute() {
    m_lights.setOnboardLights(m_color);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("LEDOnboardLightCommand end - interrupted = " + interrupted);
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "LEDOnboardLightCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "LEDOnboardLightCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }
}
