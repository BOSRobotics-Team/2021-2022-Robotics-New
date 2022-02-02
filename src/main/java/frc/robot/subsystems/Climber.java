// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final WPI_TalonFX _climberController = new WPI_TalonFX(10);
    private final WPI_TalonFX _leftPivotLinkController = new WPI_TalonFX(11);
    private final WPI_TalonFX _rightPivotLinkController = new WPI_TalonFX(12);

    private final SmartMotorHelper smartClimberController = new SmartMotorHelper(_climberController, InvertType.None);
    private final SmartMotorHelper smartPivotLinkController = new SmartMotorHelper(_rightPivotLinkController, InvertType.None, _leftPivotLinkController, InvertType.InvertMotorOutput);

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

    private final GearRatios kGearRatio_Climber;
	private final GearRatios kGearRatio_PivotLink;

    public final Gains kGains_Climber = new Gains( 0.1, 0.0, 0.0, 0.0, 100, 0.80 );
	public final Gains kGains_PivotLink = new Gains( 0.1, 0.0, 0.0, 0.0, 100, 0.80 );
    
    public Climber() {
        Preferences.initDouble("ClimberGearRatio", 12.0);
        Preferences.initDouble("ClimberWheelRadius", 1.0);
        Preferences.initDouble("ClimberPulleyRatio", 1.0);
        Preferences.initDouble("PivotLinkGearRatio", 20.0);
        Preferences.initDouble("PivotLinkWheelRadius", 1.0);
        Preferences.initDouble("PivotLinkPulleyRatio", 1.0);
        Preferences.initDouble("ClimberFeedFwd", 0.5);

        kGearRatio_Climber = new GearRatios(Preferences.getDouble("ClimberGearRatio", 12.0), 
                                            Preferences.getDouble("ClimberWheelRadius", 1.0), 
                                            Preferences.getDouble("ClimberPulleyRatio", 1.0));
        kGearRatio_PivotLink = new GearRatios(Preferences.getDouble("PivotLinkGearRatio", 20.0), 
                                              Preferences.getDouble("PivotLinkWheelRadius", 1.0), 
                                              Preferences.getDouble("PivotLinkPulleyRatio", 1.0));
        //_climberFeedFwd = Preferences.getDouble("ClimberFeedFwd", 0.1);

        smartClimberController.initController();
        smartClimberController.configureRatios(kGearRatio_Climber);
        smartClimberController.enableBrakes(true);
        smartClimberController.setDistanceConfigs(kGains_Climber);
        smartClimberController.resetPosition();

        smartPivotLinkController.initController();
        smartPivotLinkController.configureRatios(kGearRatio_PivotLink);
        smartPivotLinkController.enableBrakes(true);
        smartPivotLinkController.setDistanceConfigs(kGains_PivotLink);
        smartPivotLinkController.resetPosition();
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
        if (_isResetClimber) {
            if (isClimberRevLimitSwitchClosed()) {
                _isResetClimber = false;
                _isClimbing = false;
                smartClimberController.set(0.0);
                smartClimberController.resetPosition();
            }
        }
        if (_isClimbing) {
            System.out.println("isClimbing - current height = " + smartClimberController.getPosition() + " pos = " + smartClimberController.getNativePosition());
            smartClimberController.setTarget(_targetHeight); //, _climberFeedFwd);

            if (Math.abs(smartClimberController.getPosition() - _targetHeight) < 0.01) {
                _isClimbing = false;
                System.out.println("isClimbing - done");
            } else if (isClimberFwdLimitSwitchClosed()) {
                _isClimbing = false;
                System.out.println("isClimbing - limit exceeded");
            }
        }
        if (_isResetPivoting) {
            if (isPivotLinkRevLimitSwitchClosed()) {
                _isResetPivoting = false;
                _isPivoting = false;
                smartPivotLinkController.set(0.0);
                smartPivotLinkController.resetPosition();
            }
        }
        if (_isPivoting) {
            System.out.println("isPivoting - current distance = " + smartPivotLinkController.getPosition() + smartPivotLinkController.getNativePosition());
            smartPivotLinkController.setTarget(_targetPivot); //, _climberFeedFwd);

            if (Math.abs(smartPivotLinkController.getPosition() - _targetPivot) < 0.01) {
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
        smartClimberController.update();
        smartPivotLinkController.update();
 
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
            smartClimberController.set(speed);
        }
    }

    public void resetClimber() {
        _isResetClimber = true;
        smartClimberController.set(kResetClimberSpeed);
    }

    public void runPivotLink(double distance) {
        _targetPivot = distance;
        _isPivoting = true;

        System.out.println("isPivoting - target (meters) = " + _targetPivot);
    }
    public boolean isPivoting() { return _isPivoting; }
    public void setPivotLink(double speed) {
        if (!_isPivoting && !_isResetPivoting) {
            smartPivotLinkController.set(speed);
        }
    }

    public void resetPivotLink() {
        _isResetPivoting = true;
        smartPivotLinkController.set(kResetClimberSpeed);
    }
    public boolean isResetting() { return _isResetClimber || _isResetPivoting; }
    public boolean isClimberFwdLimitSwitchClosed() { return smartClimberController.isFwdLimitSwitchClosed() || _isFwdLimitSwitchTest; }
    public boolean isClimberRevLimitSwitchClosed() { return smartClimberController.isRevLimitSwitchClosed() || _isRevLimitSwitchTest; }
    public boolean isPivotLinkFwdLimitSwitchClosed() { return smartPivotLinkController.isFwdLimitSwitchClosed() || _isFwdLimitSwitchTest; }
    public boolean isPivotLinkRevLimitSwitchClosed() { return smartPivotLinkController.isRevLimitSwitchClosed() || _isRevLimitSwitchTest; }

    public void tripFwdLimitSwitches_test(boolean trip) {
        _isFwdLimitSwitchTest = trip;
    } 
    public void tripRevLimitSwitches_test(boolean trip) {
        System.out.println("tripRevLimitSwitches_test -" + trip);
        _isRevLimitSwitchTest = trip;
    } 
}

