// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class PivotLinkAnglePctCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  private final double m_anglePct;

  public PivotLinkAnglePctCommand(RobotContainer container, double anglePct) {
    m_climber = container.climber;
    m_anglePct = anglePct;

    addRequirements(m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker("PivotLinkAnglePctCommand", EventImportance.kNormal);
    m_climber.setPivotLinkAnglePct(m_anglePct);

    System.out.println(this.getName());
    System.out.println("PivotLinkAnglePctCommand : anglePct = " + m_anglePct);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker("PivotLinkAnglePctCommand Interrupted!", EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker("PivotLinkAnglePctCommand end", EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !m_climber.isPivoting();
  }
}
