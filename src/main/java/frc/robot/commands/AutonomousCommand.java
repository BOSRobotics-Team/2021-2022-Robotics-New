// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDLights.LEDColor;

public class AutonomousCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  private final LEDLights m_lights;

  public AutonomousCommand(RobotContainer container) {
    Preferences.initDouble("AutonomousDistance1", -2.0);

    m_driveTrain = container.driveTrain;
    m_lights = container.lights;

    addRequirements(m_driveTrain, m_lights);

    double distance1 = Preferences.getDouble("AutonomousDistance1", -2.0);
    addCommands(
        new LEDOnboardLightCommand(container, LEDColor.kYellow),
        new AutoDriveStraightCommand(container, distance1),
        new LEDOnboardLightCommand(container, LEDColor.kOff));
  }
}
