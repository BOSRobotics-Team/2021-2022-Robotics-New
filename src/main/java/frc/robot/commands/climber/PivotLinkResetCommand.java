// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class PivotLinkResetCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  public PivotLinkResetCommand(RobotContainer container) {
    m_climber = container.climber;

    addRequirements(m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "PivotLinkResetCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    m_climber.resetPivotLink();
    System.out.println("PivotLinkResetCommand - init");
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("PivotLinkResetCommand - end : interrupted = " + interrupted);

    if (interrupted) {
      Shuffleboard.addEventMarker(
          "PivotLinkResetCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "PivotLinkResetCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !m_climber.isResettingPivot();
  }
}
