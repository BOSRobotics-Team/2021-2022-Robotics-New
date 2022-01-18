// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.SmartMotor;


public class Climber extends SubsystemBase {
    private final SmartMotor _climberController = new SmartMotor(10);
    private final SmartMotor _leftPivotLinkController = new SmartMotor(11);
    private final SmartMotor _rightPivotLinkController = new SmartMotor(12);
    private boolean _isResetClimber = false;
    private boolean _isClimbing = false;
    private double _targetHeight = 0;
    private double _climberFeedFwd = 0.1;
    
    public Climber() {
        double climberGearRatio = Preferences.getDouble("ClimberGearRatio", 12.0);
        double climberWheelRadius = Preferences.getDouble("ClimberWheelRadius", 1.0);
        double pivotArmGearRatio = Preferences.getDouble("PivotArmGearRatio", 20.0);
        double pivotArmWheelRadius = Preferences.getDouble("PivotArmWheelRadius", 1.0);

        double climberHeight1 = Preferences.getDouble("ClimberHeight1", 6.0);
        double climberHeight2 = Preferences.getDouble("ClimberHeight2", 8.0);

        _climberController.configureRatios(climberGearRatio, climberWheelRadius);
        _climberController.setName("Climber");
        _climberController.configFactoryDefault();
        _climberController.enableBrakes(true);
        _climberController.setDistanceConfigs(Constants.kGains_Distanc);

        _climberController.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        _climberController.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        _climberController.resetPosition();

        _leftPivotLinkController.configureRatios(pivotArmGearRatio, pivotArmWheelRadius);
        _leftPivotLinkController.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        _leftPivotLinkController.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        _leftPivotLinkController.resetPosition();

        _rightPivotLinkController.configureRatios(pivotArmGearRatio, pivotArmWheelRadius);
        _rightPivotLinkController.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        _rightPivotLinkController.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        _rightPivotLinkController.resetPosition();

        _rightPivotLinkController.follow(_leftPivotLinkController);
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
            _climberController.setTarget(_targetHeight, _climberFeedFwd);
            if (_climberController.getPosition() == _targetHeight) {
                _isClimbing = false;
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
        _climberFeedFwd = Preferences.getDouble("ClimberFeedFwd", 0.1);
        _targetHeight = height;
        _isClimbing = true;
    }

    public void resetClimber() {
        _isResetClimber = true;
        _climberController.set(-0.2);
    }
}

