// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousCommand extends CommandBase {
@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final RobotContainer _robot;
    private final DriveTrain _driveTrain;

    double _lockedDistance = 0;
	double _targetAngle = 0;
	int _smoothing;

    public AutonomousCommand(RobotContainer container) {
        _robot = container;
        _driveTrain = _robot.driveTrain;

		addRequirements(_driveTrain);
    }
   
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        System.out.println("AutonomousCommand - initialize");

        _driveTrain.enableDriveTrain(false);
        _driveTrain.enableBrakes(false);
        _driveTrain.configForPID2();
		_driveTrain.resetPosition();
		_driveTrain.setHeadingDegrees(0);
		
		SmartDashboard.putNumber("Smoothing", _smoothing);

        _targetAngle = _driveTrain.getRightAuxPos();
        _lockedDistance = _driveTrain.getRightPos();
   
        System.out.println("_lockedDistance = " + _lockedDistance + " _targetAngle = " + _targetAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        System.out.println("AutonomousCommand - execute");

		/* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
		_driveTrain.setTarget2(_lockedDistance, _targetAngle);
		
		SmartDashboard.putNumber("PoseX", _driveTrain.getCurrentPose().getTranslation().getX());
		SmartDashboard.putNumber("PoseY", _driveTrain.getCurrentPose().getTranslation().getY());
		SmartDashboard.putNumber("Pose Rot", _driveTrain.getCurrentPose().getRotation().getDegrees());
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("AutonomousCommand - end");
        _driveTrain.enableDriveTrain(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return _driveTrain.isTargetReached();
    }
}    