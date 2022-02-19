// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class PivotLinkRetractCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  public PivotLinkRetractCommand(RobotContainer container) {
    m_climber = container.climber;

    addRequirements(m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "PivotLinkRetractCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    m_climber.setPivotLinkAnglePct(0.0);
    System.out.println("PivotLinkRetractCommand - init : distance = 0.0");
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("PivotLinkRetractCommand end - interrupted = " + interrupted);
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "PivotLinkRetractCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "PivotLinkRetractCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !m_climber.isPivoting();
  }
}
