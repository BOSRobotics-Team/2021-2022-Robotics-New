// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class AutoDriveTurnCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

	private final RobotContainer _robot;
	private final DriveTrain _driveTrain;
	private final XboxController _controller;

	private double _lockedDistance = 0;
	private double _targetAngle = 0;
	private boolean _gotoEnd = true;

	private int _smoothing = 0;

	public AutoDriveTurnCommand(RobotContainer container) {
        _robot = container;
		_driveTrain = _robot.driveTrain;
		_controller = _robot.driverController;

		addRequirements(_driveTrain);
    }
   
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        System.out.println("AutoDriveTurnCommand - initialize");

        _driveTrain.enableDriveTrain(false);
        _driveTrain.enableBrakes(false);
		_driveTrain.configForPID2();
		_driveTrain.resetPosition();
		_driveTrain.setHeadingDegrees(0);
		
		_lockedDistance = 5.0;
		_targetAngle = -900.0;  // -360000 * (90.0 / 360.0) * 0.5; //175 degrees
		_gotoEnd = true;

		SmartDashboard.putNumber("Smoothing", _smoothing);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
		System.out.println("AutoDriveTurnCommand - execute");

		if (_controller.getStartButtonPressed()) {
			_gotoEnd = !_gotoEnd;
			_lockedDistance = _gotoEnd ? 5.0 : 0.0;
			_targetAngle = _gotoEnd ? -900.0 : 900.0; //-360000 * (90.0 / 360.0) : 0.0;
		}
		if (_controller.getLeftBumperPressed()) {
			if (--_smoothing < 0) _smoothing = 0; // Cap smoothing
			_driveTrain.configMotionSCurveStrength(_smoothing);
		}
		if (_controller.getRightBumperPressed()) {
			if (++_smoothing > 8) _smoothing = 8; // Cap smoothing
			_driveTrain.configMotionSCurveStrength(_smoothing);
		}

		_driveTrain.setTarget2(_lockedDistance, _targetAngle);
		
		SmartDashboard.putNumber("PoseX", _driveTrain.getCurrentPose().getTranslation().getX());
		SmartDashboard.putNumber("PoseY", _driveTrain.getCurrentPose().getTranslation().getY());
		SmartDashboard.putNumber("Pose Rot", _driveTrain.getCurrentPose().getRotation().getDegrees());
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("AutoDriveTurnCommand - end");
        _driveTrain.enableDriveTrain(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return _driveTrain.isTargetReached();
    }
}    