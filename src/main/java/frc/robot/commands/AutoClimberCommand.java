// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDLights.LEDColor;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoClimberCommand extends SequentialCommandGroup {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;
    private final LEDLights m_lights;

    public AutoClimberCommand(Climber climber) {
        Preferences.initDouble("ClimberHeight1", 1.5);
        Preferences.initDouble("ClimberHeightIncrement", 0.1);
        Preferences.initDouble("PivotLinkDistance1", 1.0);
        Preferences.initDouble("PivotLinkDistance2", 0.5);

        m_climber = climber;
        m_lights = RobotContainer.getInstance().getLEDLights();
        addRequirements(m_climber, m_lights);

        double height1 = Preferences.getDouble("ClimberHeight1", 1.5);
        double distance1 = Preferences.getDouble("PivotLinkDistance1", 1.0);
        double increment = Preferences.getDouble("ClimberHeightIncrement", 0.1);
        double distance2 = Preferences.getDouble("PivotLinkDistance2", 0.5);

        addCommands(
            new LEDOnboardLightCommand(m_lights, LEDColor.kRed),
            new ClimberExtendCommand(m_climber, height1),
            new PivotLinkExtendCommand(m_climber, distance1),
            new LEDOnboardLightCommand(m_lights, LEDColor.kPurple),
            new ClimberExtendCommand(m_climber, height1 + increment),
            new PivotLinkExtendCommand(m_climber, distance2),
            new LEDOnboardLightCommand(m_lights, LEDColor.kGreen),
            new ClimberRetrackCommand(m_climber),
            new LEDOnboardLightOffCommand(m_lights)
        );
    }

}
