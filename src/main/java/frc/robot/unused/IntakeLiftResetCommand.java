package frc.robot.unused;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.shuffleboard.*;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.*;
// import frc.robot.subsystems.*;

// public class IntakeLiftResetCommand extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final Intake m_intake;

//   public IntakeLiftResetCommand(RobotContainer container) {
//     m_intake = null; // conatiner.intake;

//     addRequirements(m_intake);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//     Shuffleboard.addEventMarker(
//         "IntakeLiftResetCommand",
// EventImportance.kNormal);
//     m_intake.resetLift();
//     System.out.println("IntakeLiftResetCommand");
//   }

//   // Called once after isFinished returns true
//   @Override
//   public void end(boolean interrupted) {
//     System.out.println("IntakeLiftResetCommand - end : interrupted = " + interrupted);
//     if (interrupted) {
//       Shuffleboard.addEventMarker(
//           "IntakeLiftResetCommand Interrupted!",
//
//           EventImportance.kNormal);
//     }
//     Shuffleboard.addEventMarker(
//         "IntakeLiftResetCommand end", EventImportance.kNormal);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   public boolean isFinished() {
//     return !m_intake.isResetting();
//   }
// }
