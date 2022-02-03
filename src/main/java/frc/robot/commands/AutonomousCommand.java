// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.DriveMode;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousCommand extends CommandBase {
@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final DriveTrain m_driveTrain;

    private double _lockedDistance = 0;
	private double _targetAngle = 0;

    public AutonomousCommand(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
		addRequirements(m_driveTrain);
    }
   
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("AutonomousCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        System.out.println("AutonomousCommand - initialize");

        m_driveTrain.setDriveMode(DriveMode.ARCADE);
        m_driveTrain.setUseSquares(true);
        m_driveTrain.setUseDriveScaling(false);
        m_driveTrain.enableBrakes(true);
        m_driveTrain.enableDriveTrain(false);
        m_driveTrain.configForPID();
		
        _targetAngle = 0.0; //m_driveTrain.getAuxPosition();
        _lockedDistance = 10.0; //m_driveTrain.getPosition();
		
        m_driveTrain.setTarget(_lockedDistance);//, _targetAngle);
   
        System.out.println("_lockedDistance = " + _lockedDistance + " _targetAngle = " + _targetAngle);
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
        System.out.println("AutonomousCommand - end");
        m_driveTrain.enableDriveTrain(false);

        if (interrupted) {
            Shuffleboard.addEventMarker("AutonomousCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("AutonomousCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return m_driveTrain.isTargetReached(_lockedDistance);
    }
}    