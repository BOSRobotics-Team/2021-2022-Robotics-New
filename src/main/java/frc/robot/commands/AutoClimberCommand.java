// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.*;
import frc.robot.subsystems.LEDLights.LEDColor;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoClimberCommand extends SequentialCommandGroup {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public AutoClimberCommand(RobotContainer container) {
        Preferences.initDouble("ClimberHeight1", 1.5);
        Preferences.initDouble("ClimberHeightIncrement", 0.1);
        Preferences.initDouble("PivotLinkDistance1", 1.0);
        Preferences.initDouble("PivotLinkDistance2", 0.5);

        double height1 = Preferences.getDouble("ClimberHeight1", 1.5);
        double distance1 = Preferences.getDouble("PivotLinkDistance1", 1.0);
        double increment = Preferences.getDouble("ClimberHeightIncrement", 0.1);
        double distance2 = Preferences.getDouble("PivotLinkDistance2", 0.5);

        addCommands(
            new LEDOnboardLightCommand(container, LEDColor.kRed),
            new ClimberExtendCommand(container, height1),
            new PivotLinkExtendCommand(container, distance1),
            new LEDOnboardLightCommand(container, LEDColor.kPurple),
            new ClimberExtendCommand(container, height1 + increment),
            new PivotLinkExtendCommand(container, distance2),
            new LEDOnboardLightCommand(container, LEDColor.kGreen),
            new ClimberRetrackCommand(container),
            new LEDOnboardLightOffCommand(container)
        );
    }

}
