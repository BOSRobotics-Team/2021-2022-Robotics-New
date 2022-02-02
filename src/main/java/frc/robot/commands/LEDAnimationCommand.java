// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Lights.AnimationTypes;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDAnimationCommand extends InstantCommand {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Lights m_lights;
    private final AnimationTypes m_type;

    public LEDAnimationCommand(Lights lights) {
        m_lights = lights;
        m_type = AnimationTypes.SetAll;

        addRequirements(m_lights);
    }
    public LEDAnimationCommand(Lights lights, AnimationTypes type) {
        m_lights = lights;
        m_type = type;

        addRequirements(m_lights);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("LEDAnimationCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_lights.changeAnimation(m_type);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("LEDAnimationCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("LEDAnimationCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}
