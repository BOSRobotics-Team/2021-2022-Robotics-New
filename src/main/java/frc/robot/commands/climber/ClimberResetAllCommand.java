// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class ClimberResetAllCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  private final double m_speedClimber;
  private final double m_speedPivot;

  public ClimberResetAllCommand(RobotContainer container, double speedC, double speedP) {
    m_climber = container.climber;
    m_speedClimber = speedC;
    m_speedPivot = speedP;
    addRequirements(m_climber);
  }

  public ClimberResetAllCommand(RobotContainer container) {
    this(container, Constants.kResetClimberSpeed, Constants.kResetPivotSpeed);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker("ClimberResetAllCommand", EventImportance.kNormal);
    m_climber.reset(m_speedClimber, m_speedPivot);
    System.out.println(this.getName());
    System.out.println("ClimberResetAllCommand");
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker("ClimberResetAllCommand Interrupted!", EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker("ClimberResetAllCommand end", EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !m_climber.isResetting();
  }
}
