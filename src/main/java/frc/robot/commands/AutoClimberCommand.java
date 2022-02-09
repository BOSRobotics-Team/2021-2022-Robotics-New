// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;
import frc.robot.subsystems.LEDLights.AnimationTypes;
import frc.robot.subsystems.LEDLights.LEDColor;

public class AutoClimberCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public AutoClimberCommand(RobotContainer container) {
    Preferences.initDouble("ClimberHeight1", 1.0);
    Preferences.initDouble("ClimberHeight2", 0.8);
    Preferences.initDouble("PivotLinkDistance1", 0.0);
    Preferences.initDouble("ClimberHeight3", 0.1);
    Preferences.initDouble("PivotLinkDistance2", 1.0);
    Preferences.initDouble("ClimberHeight4", 0.2);
    Preferences.initDouble("ClimberHeight5", 0.0);

    double height1 = Preferences.getDouble("ClimberHeight1", 1.0);
    double height2 = Preferences.getDouble("ClimberHeight2", 0.8);
    double distance1 = Preferences.getDouble("PivotLinkDistance1", 0.0);
    double height3 = Preferences.getDouble("ClimberHeight3", 0.1);
    double distance2 = Preferences.getDouble("PivotLinkDistance2", 1.0);
    double height4 = Preferences.getDouble("ClimberHeight4", 0.2);
    double height5 = Preferences.getDouble("ClimberHeight5", 0.0);

    LEDColor led1 = new LEDColor(255, 255, 0, 0, 0, 1);
    LEDColor led2 = new LEDColor(255, 255, 0, 0, 1, 1);
    LEDColor led3 = new LEDColor(255, 255, 0, 0, 2, 1);
    LEDColor led4 = new LEDColor(255, 0, 0, 0, 3, 1);
    LEDColor led5 = new LEDColor(255, 0, 0, 0, 4, 1);
    LEDColor led6 = new LEDColor(255, 0, 0, 0, 5, 1);
    LEDColor led7 = new LEDColor(255, 255, 0, 0, 6, 1);
    LEDColor led8 = new LEDColor(255, 255, 0, 0, 7, 1);

    addCommands(
        new LEDAnimationCommand(container, AnimationTypes.Strobe, LEDColor.kYellow),
        new LEDOnboardLightCommand(container, led1),
        new ClimberExtendCommand(container, height1),
        new LEDOnboardLightCommand(container, led2),
        new ClimberExtendCommand(container, height2),
        new LEDOnboardLightCommand(container, led3),
        new PivotLinkExtendCommand(container, distance1),
        new LEDOnboardLightCommand(container, led4),
        new ClimberExtendCommand(container, height3),
        new LEDOnboardLightCommand(container, led5),
        new PivotLinkExtendCommand(container, distance2),
        new LEDOnboardLightCommand(container, led6),
        new ClimberExtendCommand(container, height4),
        new LEDOnboardLightCommand(container, led7),
        new ClimberExtendCommand(container, height5),
        new LEDOnboardLightCommand(container, led8));
  }
}
