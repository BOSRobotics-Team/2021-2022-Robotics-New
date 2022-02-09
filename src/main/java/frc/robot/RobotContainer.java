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
import frc.robot.commands.climber.CommandClimber;
import frc.robot.commands.drivetrain.AutoDriveStraightCommand;
import frc.robot.commands.drivetrain.AutoDriveTurnCommand;
import frc.robot.commands.drivetrain.CommandDriveTrain;
import frc.robot.commands.ledlights.CommandLights;
import frc.robot.subsystems.*;

// import oi.limelightvision.limelight.frc.LimeLight;

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

  // //Driver Controller
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  // Camera
  // public final LimeLight limeLight = new LimeLight();
  public UsbCamera cam0;
  public UsbCamera cam1;
  public UsbCamera cam2;

  public final AutonomousCommand m_autoCommand = new AutonomousCommand(this);
  public final AutoDriveStraightCommand m_autoDriveStraightCommand =
      new AutoDriveStraightCommand(this, 10.0);
  public final AutoDriveTurnCommand m_autoDriveTurnCommand =
      new AutoDriveTurnCommand(this, 10.0, 45.0);
  // public final DrivePathWeaverCommand m_autoWeaverCommand = new DrivePathWeaverCommand(this,
  // "paths/PathWeaver/Paths/BarrelRun");

  public final CommandDriveTrain m_cmdDriveTrainCommand = new CommandDriveTrain(this);
  public final CommandClimber m_cmdClimberCommand = new CommandClimber(this);
  // public final CommandIntake m_cmdIntakeCommand = new CommandIntake(this);
  public final CommandLights m_cmdLightsCommand = new CommandLights(this);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain.setDefaultCommand(m_cmdDriveTrainCommand);
    climber.setDefaultCommand(m_cmdClimberCommand);
    // intake.setDefaultCommand(m_cmdIntakeCommand);
    lights.setDefaultCommand(m_cmdLightsCommand);

    // Add commands to Autonomous Sendable Chooser
    chooser.setDefaultOption("Autonomous Command", m_autoCommand);
    chooser.addOption("AutoDriveStraight Command", m_autoDriveStraightCommand);
    chooser.addOption("AutoDriveTurn Command", m_autoDriveTurnCommand);
    // chooser.addOption("PathWeaver Command", m_autoWeaverCommand);

    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", m_autoCommand);
    SmartDashboard.putData("Autonomous AutoDriveStraight", m_autoDriveStraightCommand);
    SmartDashboard.putData("Autonomous AutoDriveTurn", m_autoDriveTurnCommand);
    // SmartDashboard.putData("PathWeaver Command", m_autoWeaverCommand);
    SmartDashboard.putData("CommandDriveTrain", m_cmdDriveTrainCommand);
    SmartDashboard.putData("Auto mode", chooser);

    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);

    if (RobotBase.isReal()) {
      cam0 = CameraServer.startAutomaticCapture(0);
      cam1 = CameraServer.startAutomaticCapture(1);
      cam2 = CameraServer.startAutomaticCapture(2);
    }
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
