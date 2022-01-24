// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Gains;
import frc.robot.wrappers.SmartMotor;

import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    private final SmartMotor _climberController = new SmartMotor(10);
    private final SmartMotor _leftPivotLinkController = new SmartMotor(11);
    //private final SmartMotor _rightPivotLinkController = new SmartMotor(12);
    private boolean _isResetClimber = false;
    private boolean _isResetPivoting = false;
    private boolean _isClimbing = false;
    private boolean _isPivoting = false;
    private boolean _isFwdLimitSwitchTest = false;
    private boolean _isRevLimitSwitchTest = false;
    private double _targetHeight = 0;
    private double _targetPivot = 0;
    //private double _climberFeedFwd = 0.1;
    private double kResetClimberSpeed = -0.2;

    public static final Gains kGains_Climber = new Gains( 0.1, 0.0, 0.0, 0.0, 100, 0.80 );
	public static final Gains kGains_PivotLink = new Gains( 0.1, 0.0, 0.0, 0.0, 100, 0.80 );
    
    public Climber() {
        Preferences.initDouble("ClimberGearRatio", 12.0);
        Preferences.initDouble("ClimberWheelRadius", 1.0);
        Preferences.initDouble("PivotLinkGearRatio", 20.0);
        Preferences.initDouble("PivotLinkWheelRadius", 1.0);
        Preferences.initDouble("ClimberFeedFwd", 0.5);

        double climberGearRatio = Preferences.getDouble("ClimberGearRatio", 12.0);
        double climberWheelRadius = Preferences.getDouble("ClimberWheelRadius", 1.0);
        double pivotArmGearRatio = Preferences.getDouble("PivotLinkGearRatio", 20.0);
        double pivotArmWheelRadius = Preferences.getDouble("PivotLinkWheelRadius", 1.0);

        //_climberFeedFwd = Preferences.getDouble("ClimberFeedFwd", 0.1);
        System.out.println("_climberController - ratios = " + climberGearRatio + "  radius = " + climberWheelRadius);

        _climberController.configureRatios(climberGearRatio, climberWheelRadius);
        _climberController.setName("Climber");
        _climberController.enableBrakes(true);
        _climberController.setDistanceConfigs(kGains_Climber);
        _climberController.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        _climberController.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        _climberController.resetPosition();

        _leftPivotLinkController.configureRatios(pivotArmGearRatio, pivotArmWheelRadius);
        _leftPivotLinkController.setName("LeftPivotLink");
        _leftPivotLinkController.enableBrakes(true);
        _leftPivotLinkController.setDistanceConfigs(kGains_PivotLink);
        _leftPivotLinkController.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        _leftPivotLinkController.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        _leftPivotLinkController.resetPosition();

        //_rightPivotLinkController.setName("RightPivotLink");
        //_rightPivotLinkController.follow(_leftPivotLinkController);
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
        if (_isResetClimber) {
            if (isClimberRevLimitSwitchClosed()) {
                _isResetClimber = false;
                _isClimbing = false;
                _climberController.set(0.0);
                _climberController.resetPosition();
            }
        }
        if (_isClimbing) {
            System.out.println("isClimbing - current height = " + _climberController.getPosition() + " pos = " + _climberController.getSelectedSensorPosition(0));
            _climberController.setTarget(_targetHeight); //, _climberFeedFwd);

            if (Math.abs(_climberController.getPosition() - _targetHeight) < 0.001) {
                _isClimbing = false;
                System.out.println("isClimbing - done");

                if (_targetPivot > 0.0) {
                    runPivotLink(_targetPivot);
                }
            } else if (isClimberFwdLimitSwitchClosed()) {
                _isClimbing = false;
                System.out.println("isClimbing - limit exceeded");
            }
        }
        if (_isResetPivoting) {
            if (isPivotLinkRevLimitSwitchClosed()) {
                _isResetPivoting = false;
                _isPivoting = false;
                _leftPivotLinkController.set(0.0);
                _leftPivotLinkController.resetPosition();
            }
        }
        if (_isPivoting) {
            System.out.println("isPivoting - current distance = " + _leftPivotLinkController.getPosition() + _leftPivotLinkController.getSelectedSensorPosition(0));
            _leftPivotLinkController.setTarget(_targetPivot); //, _climberFeedFwd);

            if (Math.abs(_leftPivotLinkController.getPosition() - _targetPivot) < 0.001) {
                _isPivoting = false;
                System.out.println("isPivoting - done");
            } else if (isPivotLinkFwdLimitSwitchClosed()) {
                _isPivoting = false;
                System.out.println("isPivoting - limit exceeded");
            }
        }
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void logPeriodic() {
        _climberController.logPeriodic();
        _leftPivotLinkController.logPeriodic();
        // _rightPivotLinkController.logPeriodic();
 
        SmartDashboard.putBoolean("isResetClimber", _isResetClimber);
        SmartDashboard.putBoolean("isResetPivoting", _isResetPivoting);
        SmartDashboard.putBoolean("isClimbing", _isClimbing);
        SmartDashboard.putBoolean("isPivoting", _isPivoting);
        SmartDashboard.putNumber("targetHeight", _targetHeight);
        SmartDashboard.putNumber("targetPivot", _targetPivot);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void runClimber(double height) {
        _targetHeight = height;
        _isClimbing = true;

        System.out.println("isClimbing - target (meters) = " + _targetHeight);
    }
    public boolean isClimbing() { return _isClimbing; }
    public void setClimber(double speed) {
        if (!_isClimbing && !_isResetClimber) {
            _climberController.set(speed);
        }
    }

    public void resetClimber() {
        _isResetClimber = true;
        _climberController.set(kResetClimberSpeed);
    }

    public void runPivotLink(double distance) {
        _targetPivot = distance;
        _isPivoting = true;

        System.out.println("isPivoting - target (meters) = " + _targetPivot);
    }
    public boolean isPivoting() { return _isPivoting; }
    public void setPivotLink(double speed) {
        if (!_isPivoting && !_isResetPivoting) {
            _leftPivotLinkController.set(speed);
        }
    }

    public void resetPivotLink() {
        _isResetPivoting = true;
        _leftPivotLinkController.set(kResetClimberSpeed);
    }
    public boolean isResetting() { return _isResetClimber || _isResetPivoting; }
    public boolean isClimberFwdLimitSwitchClosed() { return (_climberController.isFwdLimitSwitchClosed() == 1) || _isFwdLimitSwitchTest; }
    public boolean isClimberRevLimitSwitchClosed() { return (_climberController.isRevLimitSwitchClosed() == 1) || _isRevLimitSwitchTest; }
    public boolean isPivotLinkFwdLimitSwitchClosed() { return (_leftPivotLinkController.isFwdLimitSwitchClosed() == 1) || _isFwdLimitSwitchTest; }
    public boolean isPivotLinkRevLimitSwitchClosed() { return (_leftPivotLinkController.isRevLimitSwitchClosed() == 1) || _isRevLimitSwitchTest; }


    public void tripFwdLimitSwitches_test(boolean trip) {
        _isFwdLimitSwitchTest = trip;
    } 
    public void tripRevLimitSwitches_test(boolean trip) {
        System.out.println("tripRevLimitSwitches_test -" + trip);
        _isRevLimitSwitchTest = trip;
    } 
}

