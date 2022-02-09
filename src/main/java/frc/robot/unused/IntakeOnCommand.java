package frc.robot.unused;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.shuffleboard.*;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.*;
// import frc.robot.subsystems.*;

// public class IntakeOnCommand extends InstantCommand {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final Intake m_intake;

//   private final double m_speed;

//   public IntakeOnCommand(RobotContainer container, double speed) {
//     m_intake = null; // conatiner.intake;
//     m_speed = speed;

//     addRequirements(m_intake);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//     Shuffleboard.addEventMarker(
//         "IntakeOnCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
//     System.out.println("IntakeOnCommand - init : speed = " + m_speed);
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   public void execute() {
//     m_intake.runIntake(m_speed);
//   }

//   // Called once after isFinished returns true
//   @Override
//   public void end(boolean interrupted) {
//     System.out.println("IntakeOnCommand - end : interrupted = " + interrupted);
//     if (interrupted) {
//       Shuffleboard.addEventMarker(
//           "IntakeOnCommand Interrupted!", this.getClass().getSimpleName(),
// EventImportance.kNormal);
//     }
//     Shuffleboard.addEventMarker(
//         "IntakeOnCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
//   }
// }
