// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.DriveMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousCommand extends CommandBase {
@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final DriveTrain m_driveTrain;

    double _lockedDistance = 0;
	double _targetAngle = 0;
	int _smoothing = 0;

    public AutonomousCommand(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;

		addRequirements(m_driveTrain);
    }
   
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        System.out.println("AutonomousCommand - initialize");

        m_driveTrain.setDriveMode(DriveMode.ARCADE);
        m_driveTrain.setUseSquares(true);
        m_driveTrain.setUseDriveScaling(false);
        m_driveTrain.enableBrakes(true);
        m_driveTrain.enableDriveTrain(false);
        m_driveTrain.configForPID2();
		
		SmartDashboard.putNumber("Smoothing", _smoothing);

        _targetAngle = m_driveTrain.getAuxPosition();
        _lockedDistance = m_driveTrain.getPosition();
   
        System.out.println("_lockedDistance = " + _lockedDistance + " _targetAngle = " + _targetAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        System.out.println("AutonomousCommand - execute");

		/* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
		m_driveTrain.setTarget(_lockedDistance, _targetAngle);
		
		SmartDashboard.putNumber("PoseX", m_driveTrain.getCurrentPose().getTranslation().getX());
		SmartDashboard.putNumber("PoseY", m_driveTrain.getCurrentPose().getTranslation().getY());
		SmartDashboard.putNumber("Pose Rot", m_driveTrain.getCurrentPose().getRotation().getDegrees());
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("AutonomousCommand - end");
        m_driveTrain.enableDriveTrain(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return m_driveTrain.isTargetReached();
    }
}    