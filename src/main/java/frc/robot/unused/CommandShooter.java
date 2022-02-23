// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.unused;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.shuffleboard.*;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.*;
// import frc.robot.unused.Shooter;

// public class CommandShooter extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   public final Shooter m_shooter;

//   public final XboxController m_controller;
//   public final JoystickButton m_buttons[] = new JoystickButton[11];

//   public CommandShooter(RobotContainer container) {
//     System.out.println("CommandShooter constructor ");
//     m_shooter = container.shooter;
//     m_controller = container.getDriverController();

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_shooter);

//     // m_buttons[Button.kY.value] = new JoystickButton(m_controller, Button.kY.value);
//     // m_buttons[Button.kY.value].whenPressed(() -> m_shooter.resetShooter());

//     // m_buttons[Button.kX.value] = new JoystickButton(m_controller, Button.kX.value);
//     // m_buttons[Button.kX.value].whenPressed(() -> m_shooter.releaseShooter()));
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//     System.out.println("CommandShooter - initialize");
//     Shuffleboard.addEventMarker(
//         "CommandShooter init", EventImportance.kNormal);
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   public void execute() {}

//   // Called once after isFinished returns true
//   @Override
//   public void end(boolean interrupted) {
//     System.out.println("CommandShooter end - interrupted = " + interrupted);
//     if (interrupted) {
//       Shuffleboard.addEventMarker(
//           "CommandShooter Interrupted!",
// EventImportance.kNormal);
//     }
//     Shuffleboard.addEventMarker(
//         "CommandShooter end", EventImportance.kNormal);
//   }
// }
