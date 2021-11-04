// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class AutoDriveStraightCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

	private final RobotContainer _robot;
	private final DriveTrain _driveTrain;
    private final XboxController _controller;

	/** Config Objects for motor controllers */
	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

	private double _lockedDistance = 0;
	private boolean _gotoEnd = true;

	private int _smoothing = 0;

	public AutoDriveStraightCommand(RobotContainer container) {
        _robot = container;
		_driveTrain = _robot.driveTrain;
		_controller = _robot.driverController;

		addRequirements(_driveTrain);
    }
   
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        System.out.println("AutoDriveStraightCommand - initialize");

        _driveTrain.enableDriveTrain(false);
        _driveTrain.enableBrakes(false);

        _driveTrain.leftMaster.getAllConfigs(_leftConfig);
        _driveTrain.rightMaster.getAllConfigs(_rightConfig);

        System.out.println("AutoDriveStraightCommand - leftConfig(before): " + _leftConfig);
        System.out.println("AutoDriveStraightCommand - rightConfig(before): " + _rightConfig);

		_driveTrain.leftMaster.setDistanceConfigs(_leftConfig, Constants.kGains_Distanc);
		_driveTrain.rightMaster.setDistanceConfigs(_rightConfig, Constants.kGains_Distanc);

        System.out.println("AutoDriveStraightCommand - LeftConfig(to set): " + _leftConfig);
        System.out.println("AutoDriveStraightCommand - RightConfig(to set): " + _rightConfig);
		_driveTrain.leftMaster.configAllSettings(_leftConfig);
        _driveTrain.rightMaster.configAllSettings(_rightConfig);
        
		_driveTrain.leftMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_driveTrain.leftMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, Constants.kTimeoutMs);
		_driveTrain.rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_driveTrain.rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, Constants.kTimeoutMs);

		/* Determine which slot affects which PID */
        _driveTrain.leftMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
        _driveTrain.rightMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
        
        _driveTrain.leftMaster.resetPosition();
		_driveTrain.rightMaster.resetPosition();
		
		_lockedDistance = 5.0;
		_gotoEnd = true;

		SmartDashboard.putNumber("Smoothing", _smoothing);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
		System.out.println("AutoDriveStraightCommand - execute");

		if (_controller.getBButtonPressed()) {
			_gotoEnd = !_gotoEnd;
			_lockedDistance = _gotoEnd ? 5.0 : 0.0;
		}
		if (_controller.getLeftBumperPressed()) {
			if (--_smoothing < 0) _smoothing = 0; // Cap smoothing
			_driveTrain.leftMaster.configMotionSCurveStrength(_smoothing);
			_driveTrain.rightMaster.configMotionSCurveStrength(_smoothing);
			SmartDashboard.putNumber("Smoothing", _smoothing);
		}
		if (_controller.getRightBumperPressed()) {
			if (++_smoothing > 8) _smoothing = 8; // Cap smoothing
			_driveTrain.leftMaster.configMotionSCurveStrength(_smoothing);
			_driveTrain.rightMaster.configMotionSCurveStrength(_smoothing);
			SmartDashboard.putNumber("Smoothing", _smoothing);
		}


		/* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
        _driveTrain.leftMaster.setTarget(_lockedDistance);
		_driveTrain.rightMaster.setTarget(_lockedDistance);
		_driveTrain.logPeriodic();
			
        System.out.println("target (meters) = " + _lockedDistance);
  
	}

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("AutoDriveStraightCommand - end");
        _driveTrain.enableDriveTrain(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        double error = _driveTrain.rightMaster.getClosedLoopError();
        System.out.println("AutoDriveStraightCommand - error: " + error);

        return false;
    }
}    