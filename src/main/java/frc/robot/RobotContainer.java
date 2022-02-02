// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.unused.DrivePathWeaverCommand;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import oi.limelightvision.limelight.frc.LimeLight;

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
  public final Intake intake = new Intake();
  public final Climber climber = new Climber();
  public final Lights lights = new Lights();

  // public final UnjamIntakeCommand m_unJamIntakeCommand = new UnjamIntakeCommand(intake);
  // public final IntakeOnCommand m_intakeOnCommand = new IntakeOnCommand(intake);
  // public final IntakeOffCommand m_intakeOffCommand = new IntakeOffCommand(intake);
  // public final ExtendClimberCommand m_extendClimberCommand = new ExtendClimberCommand(climber);
  // public final RetrackClimberCommand m_retractClimberCommand = new RetrackClimberCommand(climber);
  // public final ResetClimberCommand m_resetClimberCommand = new ResetClimberCommand(climber);

  // public final ExtendPivotLinkCommand m_extendPivotLinkCommand = new ExtendPivotLinkCommand(climber);
  // public final RetrackPivotLinkCommand m_retractPivotLinkCommand = new RetrackPivotLinkCommand(climber);

  // public final AutoClimberCommand m_AutoClimberCommand = new AutoClimberCommand(climber);

  // //Driver Controller
  public final XboxController driverController = new XboxController(0);
  // public final JoystickButton a_Button_Driver = new JoystickButton(driverController, 1);
  // public final JoystickButton b_Button_Driver = new JoystickButton(driverController, 2);
  // public final JoystickButton x_Button_Driver = new JoystickButton(driverController, 3);
  // public final JoystickButton y_Button_Driver = new JoystickButton(driverController, 4);
  // public final JoystickButton left_Bumper_Driver = new JoystickButton(driverController, 5);
  // public final JoystickButton right_Bumper_Driver = new JoystickButton(driverController, 6);
  // public final JoystickButton back_Button_Driver = new JoystickButton(driverController, 7);
  // public final JoystickButton start_Button_Driver = new JoystickButton(driverController, 8);
  // public final JoystickButton left_Stick_Driver = new JoystickButton(driverController, 9);
  // public final JoystickButton right_Stick_Driver = new JoystickButton(driverController, 10);
  
  // //Operator Controller
  public final XboxController operatorController = new XboxController(1);
  // public final JoystickButton a_Button_Operator = new JoystickButton(operatorController, 1);
  // public final JoystickButton b_Button_Operator = new JoystickButton(operatorController, 2);
  // public final JoystickButton x_Button_Operator = new JoystickButton(operatorController, 3);
  // public final JoystickButton y_Button_Operator = new JoystickButton(operatorController, 4);
  // public final JoystickButton left_Bumper_Operator = new JoystickButton(operatorController, 5);
  // public final JoystickButton right_Bumper_Operator = new JoystickButton(operatorController, 6);
  // public final JoystickButton back_Button_Operator = new JoystickButton(operatorController, 7);
  // public final JoystickButton start_Button_Operator = new JoystickButton(operatorController, 8);
  // public final JoystickButton left_Stick_Operator = new JoystickButton(operatorController, 9);
  // public final JoystickButton right_Stick_Operator = new JoystickButton(operatorController, 10);
  
  //Camera
  // public final LimeLight limeLight = new LimeLight();
  public final UsbCamera cam0; 
  public final UsbCamera cam1; 
  public final UsbCamera cam2; 

  public final AutonomousCommand m_autoCommand = new AutonomousCommand(driveTrain);
  public final CommandDriveTrain m_cmdDriveTrainCommand = new CommandDriveTrain(driveTrain, driverController);
  public final CommandClimber m_cmdClimberCommand = new CommandClimber(climber, operatorController);
  public final CommandIntake m_cmdIntakeCommand = new CommandIntake(intake, operatorController);
  public final CommandLights m_cmdLightsCommand = new CommandLights(lights, operatorController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(m_cmdDriveTrainCommand);
    climber.setDefaultCommand(m_cmdClimberCommand);
    intake.setDefaultCommand(m_cmdIntakeCommand);
    lights.setDefaultCommand(m_cmdLightsCommand);
    
    // Add commands to Autonomous Sendable Chooser
    chooser.setDefaultOption("Autonomous Command", m_autoCommand);
    chooser.addOption("AutoDriveStraight Command", new AutoDriveStraightCommand(driveTrain, driverController));
    chooser.addOption("AutoDriveTurn Command", new AutoDriveTurnCommand(driveTrain, driverController));
    chooser.addOption("PathWeaver Command", new DrivePathWeaverCommand( "paths/PathWeaver/Paths/BarrelRun", driveTrain));

    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command",m_autoCommand);
    SmartDashboard.putData("Autonomous AutoDriveStraight",new AutoDriveStraightCommand(driveTrain, driverController));
    SmartDashboard.putData("Autonomous AutoDrive", new AutoDriveTurnCommand(driveTrain, driverController));
    SmartDashboard.putData("DrivePathWeaverCommand", new DrivePathWeaverCommand( "paths/PathWeaver/Paths/BarrelRun", driveTrain));
    SmartDashboard.putData("CommandDriveTrain", m_cmdDriveTrainCommand);
    SmartDashboard.putData("Auto mode", chooser);

    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);

    cam0 = CameraServer.startAutomaticCapture(0);
    cam1 = CameraServer.startAutomaticCapture(1);  
    cam2 = CameraServer.startAutomaticCapture(2);  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // left_Bumper_Driver.whenPressed(() -> driveTrain.toggleDriveMode());
    // right_Bumper_Driver.whenPressed(() -> driveTrain.setUseSquares(!driveTrain.getUseSquares()));
    // // right_Bumper_Driver.whenPressed(() -> driveTrain.setMaxOutput(0.5))
    // //                    .whenReleased(() -> driveTrain.setMaxOutput(1.0));

    // left_Stick_Driver.whenPressed(() -> driveTrain.setUseDriveScaling(!driveTrain.getUseDriveScaling()));
    // right_Stick_Driver.whenPressed(() -> driveTrain.setQuickTurn(!driveTrain.getQuickTurn()));

    // b_Button_Driver.whenPressed(new DriveDistanceProfiledCommand(3, driveTrain).withTimeout(10));

    // a_Button_Driver.whileHeld(m_unJamIntakeCommand);    
    // left_Bumper_Driver.whileHeld(m_intakeCommand);

    // a_Button_Operator.whenPressed(m_extendClimberCommand);
    // b_Button_Operator.whenPressed(m_retractClimberCommand);

    // x_Button_Operator.whenPressed(m_extendPivotLinkCommand);
    // y_Button_Operator.whenPressed(m_retractPivotLinkCommand);

    // start_Button_Operator.whenPressed(m_resetClimberCommand);
    // left_Bumper_Operator.whenPressed(m_AutoClimberCommand);

    // back_Button_Operator.whenPressed(() -> climber.tripRevLimitSwitches_test(true))
    //                     .whenReleased(() -> climber.tripRevLimitSwitches_test(false));

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
}
