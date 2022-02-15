// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class ClimberRetrackCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  public ClimberRetrackCommand(RobotContainer container) {
    m_climber = container.climber;

    addRequirements(m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "ClimberRetrackCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    m_climber.setClimberHeightPct(0.0);

    System.out.println("ClimberRetrackCommand - init : height = 0.0");
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("ClimberRetrackCommand - end : interrupted = " + interrupted);

    if (interrupted) {
      Shuffleboard.addEventMarker(
          "ClimberRetrackCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "ClimberRetrackCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !m_climber.isClimbing();
  }
}
