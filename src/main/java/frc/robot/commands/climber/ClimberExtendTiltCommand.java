// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class ClimberExtendTiltCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  private final double m_height;
  private final double m_tilt;

  public ClimberExtendTiltCommand(RobotContainer container, double pctHeight, double pctTilt) {
    m_climber = container.climber;
    m_height = pctHeight;
    m_tilt = pctTilt;

    addRequirements(m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "ClimberExtendTiltCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    m_climber.setClimberHeight(m_height);
    m_climber.setPivotLinkAngle(m_tilt);

    System.out.println(
        "ClimberExtendTiltCommand - init : height = " + m_height + " tilt = " + m_tilt);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("ClimberExtendTiltCommand - end : interrupted = " + interrupted);

    if (interrupted) {
      Shuffleboard.addEventMarker(
          "ClimberExtendTiltCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "ClimberExtendTiltCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !m_climber.isClimbing() && !m_climber.isPivoting();
  }
}
