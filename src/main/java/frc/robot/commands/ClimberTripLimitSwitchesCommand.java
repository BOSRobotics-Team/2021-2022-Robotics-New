// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimberTripLimitSwitchesCommand extends InstantCommand {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;

    public ClimberTripLimitSwitchesCommand(Climber climber) {
        m_climber = climber;

        addRequirements(m_climber);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("ClimberTripLimitSwitchesCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        m_climber.tripRevLimitSwitches_test(true);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_climber.tripRevLimitSwitches_test(false);
        if (interrupted) {
            Shuffleboard.addEventMarker("ClimberTripLimitSwitchesCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("ClimberTripLimitSwitchesCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}
