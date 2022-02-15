// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class PivotLinkExtendCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  private final double m_distance;

  public PivotLinkExtendCommand(RobotContainer container, double pctDistance) {
    m_climber = container.climber;
    m_distance = pctDistance;

    addRequirements(m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "PivotLinkExtendCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    m_climber.setPivotLinkPct(m_distance);

    System.out.println("PivotLinkExtendCommand - init : distance = " + m_distance);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {}

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("PivotLinkExtendCommand end - interrupted = " + interrupted);
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "PivotLinkExtendCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "PivotLinkExtendCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !m_climber.isPivoting();
  }
}
