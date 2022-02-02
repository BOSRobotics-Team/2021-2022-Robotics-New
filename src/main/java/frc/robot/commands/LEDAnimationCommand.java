// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Lights.AnimationTypes;

public class LEDAnimationCommand extends CommandBase {
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
        m_lights.changeAnimation(m_type);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }

}
