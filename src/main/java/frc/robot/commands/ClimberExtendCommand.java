// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberExtendCommand extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;

    public ClimberExtendCommand(Climber climber) {
        m_climber = climber;

        addRequirements(m_climber);
        Preferences.initDouble("ClimberHeight1", 1.5);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        double height = Preferences.getDouble("ClimberHeight1", 1.5);

        m_climber.runClimber(height);
        System.out.println("extendClimber - height = " + height);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        // m_hook.runClimber(0);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (!m_climber.isClimbing()) {
            System.out.println("extendClimber - isFinished");
            return true;
        }
        return false;
    }
}
