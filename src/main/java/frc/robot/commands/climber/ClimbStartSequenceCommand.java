// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.*;
import frc.robot.subsystems.*;

public class ClimbStartSequenceCommand extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;

  public ClimbStartSequenceCommand(RobotContainer container) {
    m_climber = container.climber;
    addRequirements(m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker("ClimbResetSequenceCommand", EventImportance.kNormal);
    System.out.println("ClimbResetSequenceCommand");
  }

  /** Do the state change. */
  @Override
  public void execute() {
    m_climber.startClimbingSequence();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "ClimbResetSequenceCommand Interrupted!", EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker("ClimbResetSequenceCommand end", EventImportance.kNormal);
  }
}
