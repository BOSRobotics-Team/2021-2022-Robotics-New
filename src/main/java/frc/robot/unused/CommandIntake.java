package frc.robot.unused;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.shuffleboard.*;
// // import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.*;

// public class CommandIntake extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   public final Intake m_intake;

//   public final XboxController m_controller;
//   // public final JoystickButton right_Bumper;
//   // public final JoystickButton left_Stick;
//   // public final JoystickButton right_Stick;

//   private final IntakeOnCommand _onCommand;
//   private final IntakeOffCommand _offCommand;
//   // private final IntakeUnjamCommand _unjamCommand;

//   private final IntakeLiftUpCommand _upCommand;
//   private final IntakeLiftDownCommand _dnCommand;
//   // private final IntakeLiftResetCommand _resetLiftCommand;

//   private boolean _lastTriggerL = false;
//   private boolean _lastTriggerR = false;

//   public CommandIntake(RobotContainer container) {
//     m_intake = null; // container.intake;
//     m_controller = container.getOperatorController();

//     _onCommand = new IntakeOnCommand(container, Constants.kIntakeSpeed);
//     _offCommand = new IntakeOffCommand(container);
//     // _unjamCommand = new IntakeUnjamCommand(container);

//     _upCommand = new IntakeLiftUpCommand(container, 1.2);
//     _dnCommand = new IntakeLiftDownCommand(container);
//     // _resetLiftCommand = new IntakeLiftResetCommand(container);

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_intake);

//     // right_Bumper = new JoystickButton(m_controller, 6);
//     // left_Stick = new JoystickButton(m_controller, 9);
//     // right_Stick = new JoystickButton(m_controller, 10);

//     // left_Stick.whenPressed(_resetLiftCommand);
//     // right_Stick.whenPressed(_unjamCommand);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//     Shuffleboard.addEventMarker(
//         "CommandIntake init", EventImportance.kNormal);
//     _lastTriggerL = _lastTriggerR = false;
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   public void execute() {
//     double triggerL = m_controller.getLeftTriggerAxis();
//     if ((triggerL > 0.5) && !_lastTriggerL) {
//       _onCommand.schedule();
//     } else if ((triggerL <= 0.5) && _lastTriggerL) {
//       _offCommand.schedule();
//     }
//     _lastTriggerL = (triggerL > 0.5);

//     double triggerR = m_controller.getRightTriggerAxis();
//     if ((triggerR > 0.5) && !_lastTriggerR) {
//       _upCommand.schedule();
//     } else if ((triggerL <= 0.5) && _lastTriggerL) {
//       _dnCommand.schedule();
//     }
//     _lastTriggerR = (triggerR > 0.5);

//     //    m_intake.logPeriodic();
//   }

//   // Called once after isFinished returns true
//   @Override
//   public void end(boolean interrupted) {
//     if (interrupted) {
//       Shuffleboard.addEventMarker(
//           "CommandIntake Interrupted!",
// EventImportance.kNormal);
//     }
//     Shuffleboard.addEventMarker(
//         "CommandIntake end", EventImportance.kNormal);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
