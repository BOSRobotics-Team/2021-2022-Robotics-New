// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class ClimberExtendCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  private final double m_height;
  private final double m_arbFF;

  public ClimberExtendCommand(RobotContainer container, double height, double arbFF) {
    m_climber = container.climber;
    m_height = height;
    m_arbFF = arbFF;

    addRequirements(m_climber);
  }

  public ClimberExtendCommand(RobotContainer container, double height) {
    this(container, height, 0.0);
  }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker("ClimberExtendCommand", this.getName(), EventImportance.kNormal);
    m_climber.setClimberHeight(m_height, m_arbFF);

    System.out.println(this.getName());
    System.out.println("ClimberExtendCommand : height = " + m_height + " arbFF = " + m_arbFF);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker("ClimberExtendCommand Interrupted!", EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker("ClimberExtendCommand end", EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !m_climber.isClimbing();
  }
}
