// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.*;
import frc.robot.subsystems.*;

public class IntakeOffCommand extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;

  public IntakeOffCommand(RobotContainer container) {
    m_intake = null; // conatiner.intake;

    addRequirements(m_intake);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "IntakeOffCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    System.out.println("IntakeOffCommand - init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_intake.stopIntake();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("IntakeOffCommand - end : interrupted = " + interrupted);
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "IntakeOffCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "IntakeOffCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }
}
