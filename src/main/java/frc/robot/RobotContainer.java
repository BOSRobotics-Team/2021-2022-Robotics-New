// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.subsystem.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  SendableChooser<Command> chooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  public final DriveTrain driveTrain = new DriveTrain();
  // public final Intake intake = new Intake();
  public final Climber climber = new Climber();
  public final LEDLights lights = new LEDLights();
  public final Shooter shooter = new Shooter();

  // //Driver Controller
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  // Camera
  public UsbCamera cam0;
  public UsbCamera cam1;
  public UsbCamera cam2;

  public final AutonomousCommand m_autoCommand = new AutonomousCommand(this);
  public final AutoDriveStraightCommand m_autoDriveStraightCommand =
      new AutoDriveStraightCommand(this, -2.0);
  public final AutoDriveTurnCommand m_autoDriveTurnCommand =
      new AutoDriveTurnCommand(this, -2.0, -90.0);

  public final CommandDriveTrain m_cmdDriveTrainCommand = new CommandDriveTrain(this);
  public final CommandClimber m_cmdClimberCommand = new CommandClimber(this);
  public final CommandLights m_cmdLightsCommand = new CommandLights(this);
  public final CommandShooter m_cmdShooterCommand = new CommandShooter(this);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain.setDefaultCommand(m_cmdDriveTrainCommand);
    climber.setDefaultCommand(m_cmdClimberCommand);
    lights.setDefaultCommand(m_cmdLightsCommand);

    // Add commands to Autonomous Sendable Chooser
    chooser.setDefaultOption("Autonomous Command", m_autoCommand);
    chooser.addOption("AutoDriveStraight Command", m_autoDriveStraightCommand);
    chooser.addOption("AutoDriveTurn Command", m_autoDriveTurnCommand);

    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", m_autoCommand);
    SmartDashboard.putData("Autonomous AutoDriveStraight", m_autoDriveStraightCommand);
    SmartDashboard.putData("Autonomous AutoDriveTurn", m_autoDriveTurnCommand);
    SmartDashboard.putData("CommandDriveTrain", m_cmdDriveTrainCommand);
    SmartDashboard.putData("Auto mode", chooser);

    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);

    if (RobotBase.isReal()) {
      cam0 = CameraServer.startAutomaticCapture(0);
      cam1 = CameraServer.startAutomaticCapture(1);
      cam2 = CameraServer.startAutomaticCapture(2);
    }
  }

  public void logPeriodic() {
    driveTrain.logPeriodic();
    climber.logPeriodic();
    lights.logPeriodic();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
  }

  public XboxController getDriverController() {
    return driverController;
  }

  public XboxController getOperatorController() {
    return operatorController;
  }

  public LEDLights getLEDLights() {
    return lights;
  }
}
