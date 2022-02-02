// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoDriveTurnCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final DriveTrain m_driveTrain;
	private final XboxController m_controller;

	private double _lockedDistance = 0;
	private double _targetAngle = 0;
	private boolean _gotoEnd = true;

	private int _smoothing = 0;

	public AutoDriveTurnCommand(DriveTrain driveTrain, XboxController driverController) {
		m_driveTrain = driveTrain;
		m_controller = driverController;

		addRequirements(m_driveTrain);
    }
   
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        System.out.println("AutoDriveTurnCommand - initialize");

        m_driveTrain.enableDriveTrain(false);
        m_driveTrain.enableBrakes(false);
		m_driveTrain.configForPID2();
		
		_lockedDistance = 5.0;
		_targetAngle = 175; //-900.0;  // -360000 * (90.0 / 360.0) * 0.5; //175 degrees
		_gotoEnd = true;

		SmartDashboard.putNumber("Smoothing", _smoothing);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
		System.out.println("AutoDriveTurnCommand - execute");

		if (m_controller.getStartButtonPressed()) {
			_gotoEnd = !_gotoEnd;
			_lockedDistance = _gotoEnd ? 5.0 : 0.0;
			_targetAngle = _gotoEnd ? -175 : 175;       //-900.0 : 900.0; //-360000 * (90.0 / 360.0) : 0.0;
		}
		if (m_controller.getLeftBumperPressed()) {
			if (--_smoothing < 0) _smoothing = 0; // Cap smoothing
			m_driveTrain.configMotionSCurveStrength(_smoothing);
		}
		if (m_controller.getRightBumperPressed()) {
			if (++_smoothing > 8) _smoothing = 8; // Cap smoothing
			m_driveTrain.configMotionSCurveStrength(_smoothing);
		}

		m_driveTrain.setTarget(_lockedDistance, _targetAngle);
		
		SmartDashboard.putNumber("PoseX", m_driveTrain.getCurrentPose().getTranslation().getX());
		SmartDashboard.putNumber("PoseY", m_driveTrain.getCurrentPose().getTranslation().getY());
		SmartDashboard.putNumber("Pose Rot", m_driveTrain.getCurrentPose().getRotation().getDegrees());
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("AutoDriveTurnCommand - end");
        m_driveTrain.enableDriveTrain(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return m_driveTrain.isTargetReached();
    }
}    