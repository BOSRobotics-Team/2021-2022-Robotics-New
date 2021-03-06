// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class AutoDriveStraightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  private double _targetDistance = 0.0;
  private double _targetAngle = 0;
  private int _smoothing = -1;

  public AutoDriveStraightCommand(RobotContainer container, double distance, int smoothing) {
    m_driveTrain = container.driveTrain;
    _targetDistance = distance;
    _smoothing = smoothing;

    addRequirements(m_driveTrain);
  }

  public AutoDriveStraightCommand(RobotContainer container, double distance) {
    this(container, distance, -1);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker("AutoDriveStraightCommand", EventImportance.kNormal);

    /* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
    m_driveTrain.enableDriveTrain(false);
    m_driveTrain.enableBrakes(false);
    m_driveTrain.configDistanceGains(Constants.kDriveGains_Distanc);
    if (_smoothing >= 0) m_driveTrain.configMotionSCurveStrength(_smoothing);

    // m_driveTrain.setTargetAndAngle(_targetDistance, _targetAngle);
    m_driveTrain.setTarget(_targetDistance);

    System.out.println(
        "AutoDriveStraightCommand : targetDistance = "
            + _targetDistance
            + " targetAngle = "
            + _targetAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    SmartDashboard.putNumber("PoseX", m_driveTrain.getCurrentPose().getTranslation().getX());
    SmartDashboard.putNumber("PoseY", m_driveTrain.getCurrentPose().getTranslation().getY());
    SmartDashboard.putNumber("Pose Rot", m_driveTrain.getCurrentPose().getRotation().getDegrees());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.enableDriveTrain(false);

    if (interrupted) {
      Shuffleboard.addEventMarker("AutoDriveStraightCommand Interrupted!", EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker("AutoDriveStraightCommand end", EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return m_driveTrain.isTargetReached();
  }
}
