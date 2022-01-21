// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Gains;
import frc.robot.wrappers.SmartMotor;

import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    private final SmartMotor _climberController = new SmartMotor(10);
    private final SmartMotor _leftPivotLinkController = new SmartMotor(11);
    //private final SmartMotor _rightPivotLinkController = new SmartMotor(12);
    private boolean _isResetClimber = false;
    private boolean _isClimbing = false;
    private double _targetHeight = 0;
    private double _climberFeedFwd = 0.1;
    private double kResetClimberSpeed = -0.2;

    public static final Gains kGains_Climber = new Gains( 0.1, 0.0, 0.0, 0.0, 100, 0.80 );
	public static final Gains kGains_PivotArm = new Gains( 0.1, 0.0, 0.0, 0.0, 100, 0.80 );

    
    public Climber() {
        double climberGearRatio = Preferences.getDouble("ClimberGearRatio", 12.0);
        double climberWheelRadius = Preferences.getDouble("ClimberWheelRadius", 1.0);
        double pivotArmGearRatio = Preferences.getDouble("PivotArmGearRatio", 20.0);
        double pivotArmWheelRadius = Preferences.getDouble("PivotArmWheelRadius", 1.0);

        _climberFeedFwd = Preferences.getDouble("ClimberFeedFwd", 0.1);

        _climberController.configureRatios(climberGearRatio, climberWheelRadius);
        _climberController.setName("Climber");
        _climberController.enableBrakes(true);
        _climberController.setDistanceConfigs(kGains_Climber);
        _climberController.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        _climberController.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        _climberController.resetPosition();

        _leftPivotLinkController.configureRatios(pivotArmGearRatio, pivotArmWheelRadius);
        _leftPivotLinkController.setName("LeftPivotArm");
        _leftPivotLinkController.enableBrakes(true);
        _leftPivotLinkController.setDistanceConfigs(kGains_PivotArm);
        _leftPivotLinkController.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        _leftPivotLinkController.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        _leftPivotLinkController.resetPosition();

        //_rightPivotLinkController.setName("RightPivotArm");
        //_rightPivotLinkController.follow(_leftPivotLinkController);
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
        if (_isResetClimber) {
            if (_climberController.isRevLimitSwitchClosed() == 1) {
                _isResetClimber = false;
                _isClimbing = false;
                _climberController.resetPosition();
            }
        }
        if (_isClimbing) {
            System.out.println("isClimbing - current height = " + _climberController.getPosition() + _climberController.getSelectedSensorPosition(0));
            _climberController.setTarget(_targetHeight); //, _climberFeedFwd);

            if (Math.abs(_climberController.getPosition() - _targetHeight) < 0.001) {
                _isClimbing = false;
                System.out.println("isClimbing - done");
            }

            if (_climberController.isFwdLimitSwitchClosed() == 1) {
                _isClimbing = false;
                System.out.println("isClimbing - limit exceeded");
            }
        }
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void runClimber(double height) {
        _targetHeight = height;
        _isClimbing = true;

        System.out.println("isClimbing - target (meters) = " + _targetHeight);
    }

    public void resetClimber() {
        _isResetClimber = true;

        _climberController.set(kResetClimberSpeed);
    }
}

