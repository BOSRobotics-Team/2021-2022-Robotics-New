// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.subsystem;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.shuffleboard.*;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.RobotContainer;
// import frc.robot.commands.ledlights.*;
// import frc.robot.subsystems.*;

// public class CommandLights extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   public final LEDLights m_lights;

//   public final XboxController m_controller;
//   public final JoystickButton m_buttons[] = new JoystickButton[11];

//   private double _lastVal = 0.0;
//   private double _currentVal = 0.0;

//   private final LEDAnimationCommand _animationOffCommand;
//   private final LEDAnimationRotateCommand _animationRotateCommand;

//   // private final LEDOnboardLightCommand _onboardLightCommand;
//   // private final LEDOnboardLightCommand _onboardLightOffCommand;

//   // private final LEDStripLightCommand _stripLightCommand;
//   // private final LEDStripLightCommand _stripLightOffCommand;

//   public CommandLights(RobotContainer container) {
//     m_lights = container.lights;
//     m_controller = container.getDriverController();

//     _animationOffCommand = new LEDAnimationCommand(container);
//     _animationRotateCommand = new LEDAnimationRotateCommand(container, true);
//     // _onboardLightCommand = new LEDOnboardLightCommand(container, LEDColor.kWhite);
//     // _onboardLightOffCommand = new LEDOnboardLightCommand(container, LEDColor.kOff);
//     // _stripLightCommand = new LEDStripLightCommand(container, LEDColor.kWhite);
//     // _stripLightOffCommand = new LEDStripLightCommand(container, LEDColoer.kOff);

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_lights);

//     // m_buttons[Button.kY.value] = new JoystickButton(m_controller, Button.kY.value);
//     // m_buttons[Button.kY.value].whenPressed(_animationOffCommand);

//     // m_buttons[Button.kX.value] = new JoystickButton(m_controller, Button.kX.value);
//     // m_buttons[Button.kX.value].whenPressed(_animationRotateCommand);

//     SmartDashboard.putData("Animation Off", _animationOffCommand);
//     SmartDashboard.putData("Animation Rotate", _animationRotateCommand);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//     Shuffleboard.addEventMarker("CommandLights", EventImportance.kNormal);
//     _currentVal = _lastVal = 0.0;
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   public void execute() {
//     if (!m_lights.isAnimating()) {
//       _currentVal = m_controller.getLeftX();
//       if (_currentVal != _lastVal) {
//         int lx = (int) ((_currentVal + 1.0) * 0x7FFFFF);
//         m_lights.runLights((lx >> 16) & 0xFF, (lx >> 8) & 0xFF, lx & 0xFF);
//         _lastVal = _currentVal;
//       }
//     }
//   }

//   // Called once after isFinished returns true
//   @Override
//   public void end(boolean interrupted) {
//     if (interrupted) {
//       Shuffleboard.addEventMarker("CommandLights Interrupted!", EventImportance.kNormal);
//     }
//     Shuffleboard.addEventMarker("CommandLights end", EventImportance.kNormal);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
