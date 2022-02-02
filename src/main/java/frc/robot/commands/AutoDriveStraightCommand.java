// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveStraightCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

	private final DriveTrain m_driveTrain;
    private final XboxController m_controller;

	private double _lockedDistance = 0;
	private boolean _gotoEnd = true;

	private int _smoothing = 0;

	public AutoDriveStraightCommand(DriveTrain driveTrain, XboxController driverController) {
		m_driveTrain = driveTrain;
		m_controller = driverController;

		addRequirements(m_driveTrain);
    }
   
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("AutoDriveStraightCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);

        m_driveTrain.enableDriveTrain(false);
        m_driveTrain.enableBrakes(false);
		m_driveTrain.configForPID();
		
		_lockedDistance = 5.0;
		_gotoEnd = true;

		SmartDashboard.putNumber("Smoothing", _smoothing);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
		System.out.println("AutoDriveStraightCommand - execute");

		if (m_controller.getBButtonPressed()) {
			_gotoEnd = !_gotoEnd;
			_lockedDistance = _gotoEnd ? 5.0 : 0.0;
		}
		if (m_controller.getLeftBumperPressed()) {
			if (--_smoothing < 0) _smoothing = 0; // Cap smoothing
			m_driveTrain.configMotionSCurveStrength(_smoothing);
		}
		if (m_controller.getRightBumperPressed()) {
			if (++_smoothing > 8) _smoothing = 8; // Cap smoothing
			m_driveTrain.configMotionSCurveStrength(_smoothing);
		}

		/* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
        m_driveTrain.setTarget(_lockedDistance);  
	}

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("AutoDriveStraightCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("AutoDriveStraightCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
        m_driveTrain.enableDriveTrain(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return m_driveTrain.isTargetReached();
    }
}    