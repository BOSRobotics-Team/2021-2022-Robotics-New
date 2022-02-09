package frc.robot.unused;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.shuffleboard.*;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.*;
// import frc.robot.subsystems.*;

// public class IntakeUnjamCommand extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final Intake m_intake;

//   private boolean _isFinished = false;
//   private int _counter = 100;

//   public IntakeUnjamCommand(RobotContainer container) {
//     m_intake = null; // conatiner.intake;

//     addRequirements(m_intake);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//     Shuffleboard.addEventMarker(
//         "IntakeUnjamCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
//     _counter = 100;
//     m_intake.runIntake(Constants.kUnjamIntakeSpeed);

//     System.out.println("IntakeOnCommand - init");
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   public void execute() {
//     _isFinished = (_counter-- < 0);
//   }

//   // Called once after isFinished returns true
//   @Override
//   public void end(boolean interrupted) {
//     System.out.println("IntakeUnjamCommand - end : interrupted = " + interrupted);
//     m_intake.stopIntake();
//     if (interrupted) {
//       Shuffleboard.addEventMarker(
//           "IntakeUnjamCommand Interrupted!",
//           this.getClass().getSimpleName(),
//           EventImportance.kNormal);
//     }
//     Shuffleboard.addEventMarker(
//         "IntakeUnjamCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   public boolean isFinished() {
//     return _isFinished;
//   }
// }
