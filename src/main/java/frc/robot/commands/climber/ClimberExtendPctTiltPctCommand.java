// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class ClimberExtendPctTiltPctCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  private final double m_height;
  private final double m_tilt;
  private final double m_arbFF;

  public ClimberExtendPctTiltPctCommand(
      RobotContainer container, double pctHeight, double pctTilt, double arbFF) {
    m_climber = container.climber;
    m_height = pctHeight;
    m_tilt = pctTilt;
    m_arbFF = arbFF;

    addRequirements(m_climber);
  }

  public ClimberExtendPctTiltPctCommand(
      RobotContainer container, double pctHeight, double pctTilt) {
    this(container, pctHeight, pctTilt, 0.0);
  }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker("ClimberExtendPctTiltPctCommand", EventImportance.kNormal);
    m_climber.setClimberHeightPct(m_height, m_arbFF);
    m_climber.setPivotLinkAnglePct(m_tilt);

    System.out.println(
        "ClimberExtendPctTiltPctCommand : heightPct = "
            + m_height
            + " tiltPct = "
            + m_tilt
            + " arbFF = "
            + m_arbFF);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "ClimberExtendPctTiltPctCommand Interrupted!", EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker("ClimberExtendPctTiltPctCommand end", EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !m_climber.isClimbing() && !m_climber.isPivoting();
  }
}
