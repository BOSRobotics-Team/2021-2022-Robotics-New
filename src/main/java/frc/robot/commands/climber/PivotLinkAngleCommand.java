// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class PivotLinkAngleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  private final double m_angle;

  public PivotLinkAngleCommand(RobotContainer container, double angle) {
    m_climber = container.climber;
    m_angle = angle;

    addRequirements(m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker("PivotLinkAngleCommand", EventImportance.kNormal);
    m_climber.setPivotLinkAngle(m_angle);

    System.out.println("PivotLinkAngleCommand : distance = " + m_angle);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker("PivotLinkAngleCommand Interrupted!", EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker("PivotLinkAngleCommand end", EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !m_climber.isPivoting();
  }
}
