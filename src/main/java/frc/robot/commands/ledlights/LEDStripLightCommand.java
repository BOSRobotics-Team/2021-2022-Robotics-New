// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ledlights;

// import edu.wpi.first.wpilibj.shuffleboard.*;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.*;
// import frc.robot.subsystems.*;
// import frc.robot.subsystems.LEDLights.LEDColor;

// public class LEDStripLightCommand extends InstantCommand {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final LEDLights m_lights;

//   private final LEDColor m_color;

//   public LEDStripLightCommand(RobotContainer container, LEDColor color) {
//     m_lights = container.lights;
//     m_color = color;

//     addRequirements(m_lights);
//   }

//   public LEDStripLightCommand(RobotContainer container) {
//     this(container, LEDColor.kOff);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//     Shuffleboard.addEventMarker("LEDStripLightCommand", EventImportance.kNormal);
//     System.out.println("LEDStripLightCommand : color = " + m_color);
//   }

//   // Called when this Command is scheduled to run
//   @Override
//   public void execute() {
//     m_lights.setStripLights(m_color);
//   }

//   // Called once after isFinished returns true
//   @Override
//   public void end(boolean interrupted) {
//     if (interrupted) {
//       Shuffleboard.addEventMarker("LEDStripLightCommand Interrupted!", EventImportance.kNormal);
//     }
//     Shuffleboard.addEventMarker("LEDStripLightCommand end", EventImportance.kNormal);
//   }
// }
