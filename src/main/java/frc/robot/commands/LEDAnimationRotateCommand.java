// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDAnimationRotateCommand extends InstantCommand {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LEDLights m_lights;
    private final boolean m_increment;

    public LEDAnimationRotateCommand(RobotContainer container, boolean increment) {
        System.out.println("LEDAnimationRotateCommand constructor");
        m_lights = container.lights;
        m_increment = increment;

        addRequirements(m_lights);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("LEDAnimationRotateCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        System.out.println("LEDAnimationRotateCommand init : increment = " + m_increment);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        System.out.println("LEDAnimationRotateCommand - execute");
        if (m_increment)
            m_lights.incrementAnimation();
        else
            m_lights.decrementAnimation();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("LEDAnimationRotateCommand end - interrupted = " + interrupted);
        if (interrupted) {
            Shuffleboard.addEventMarker("LEDAnimationRotateCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("LEDAnimationRotateCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}
