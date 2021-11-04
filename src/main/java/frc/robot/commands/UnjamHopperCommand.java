// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class UnjamHopperCommand extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Hopper m_hopper;

    public UnjamHopperCommand(Hopper hopper) {
        m_hopper = hopper;

        addRequirements(m_hopper);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_hopper.setMotorSpeed(Constants.kUnjamHopperSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
     // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_hopper.setMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
