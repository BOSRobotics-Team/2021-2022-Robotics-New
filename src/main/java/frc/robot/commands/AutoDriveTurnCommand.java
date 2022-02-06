// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveTurnCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final DriveTrain m_driveTrain;
	private final XboxController m_controller;

	private double _targetDistance = 0;
	private double _targetAngle = 0;
	private int _smoothing = -1;

	private boolean _gotoEnd = true;

	public AutoDriveTurnCommand(RobotContainer container, double distance, double angle, int smoothing) {
		m_driveTrain = container.driveTrain;
		m_controller = container.getDriverController();

		_targetDistance = distance;
		_targetAngle = angle; //-900.0;  // -360000 * (90.0 / 360.0) * 0.5; //175 degrees
		_smoothing = smoothing;
		_gotoEnd = true;

		addRequirements(m_driveTrain);
    }
	public AutoDriveTurnCommand(RobotContainer container, double distance, double angle) {
		this(container, distance, angle, -1);
	}
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("AutoDriveTurnCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);

        m_driveTrain.enableDriveTrain(false);
        m_driveTrain.enableBrakes(false);
		m_driveTrain.configForPID2();
        if (_smoothing >= 0)
            m_driveTrain.configMotionSCurveStrength(_smoothing);
		m_driveTrain.setTarget(_targetDistance, _targetAngle);
		
		System.out.println("AutoDriveTurnCommand init : targetDistance = " + _targetDistance + " targetAngle = " + _targetAngle);
		SmartDashboard.putNumber("Smoothing", _smoothing);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
		if (m_controller.getStartButtonPressed()) {
			_gotoEnd = !_gotoEnd;
			_targetDistance = _gotoEnd ? 5.0 : 0.0;
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

		SmartDashboard.putNumber("PoseX", m_driveTrain.getCurrentPose().getTranslation().getX());
		SmartDashboard.putNumber("PoseY", m_driveTrain.getCurrentPose().getTranslation().getY());
		SmartDashboard.putNumber("Pose Rot", m_driveTrain.getCurrentPose().getRotation().getDegrees());
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("AutoDriveTurnCommand - end : interrupted = " + interrupted);
        m_driveTrain.enableDriveTrain(false);

        if (interrupted) {
            Shuffleboard.addEventMarker("AutoDriveTurnCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("AutoDriveTurnCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return m_driveTrain.isTargetReached(_targetDistance);
    }
}    