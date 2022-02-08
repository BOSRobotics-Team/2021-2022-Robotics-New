// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.util.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final WPI_TalonFX _leftclimberController = new WPI_TalonFX(9);
    private final WPI_TalonFX _rightclimberController = new WPI_TalonFX(10);
    private final WPI_TalonFX _leftPivotLinkController = new WPI_TalonFX(11);
    private final WPI_TalonFX _rightPivotLinkController = new WPI_TalonFX(12);

    private final SmartMotorController smartClimberController = new SmartMotorController(_rightclimberController, _leftclimberController);
    private final SmartMotorController smartPivotLinkController = new SmartMotorController(_rightPivotLinkController, _leftPivotLinkController);

    private final GearRatios kGearRatio_Climber;
    public final Gains kGains_Climber = new Gains( 0.1, 0.0, 0.0, 0.0, 100, 0.80 );
    private boolean _isClimbing = false;
    private boolean _isResetClimber = false;
    private double  _lastLClimberHeight = 0.0;
    private double  _lastRClimberHeight = 0.0;
    private double _targetClimberHeight = 0;
    private double kResetClimberSpeed = -0.1;
    //private double _climberFeedFwd = 0.1;

	private final GearRatios kGearRatio_PivotLink;
	public final Gains kGains_PivotLink = new Gains( 0.1, 0.0, 0.0, 0.0, 100, 0.80 );    
    private boolean _isPivoting = false;
    private boolean _isResetPivoting = false;
    private double  _lastLPivotDistance = 0.0;
    private double  _lastRPivotDistance = 0.0;
    private double _targetPivotDistance = 0;
    private double kResetPivotSpeed = -0.08;

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

        _leftclimberController.setInverted(InvertType.None);
        _rightclimberController.setInverted(InvertType.InvertMotorOutput);

        smartClimberController.initController();
        smartClimberController.configureRatios(kGearRatio_Climber);
        smartClimberController.enableBrakes(true);
        smartClimberController.setDistanceConfigs(kGains_Climber);
        smartClimberController.resetPosition();

        _leftPivotLinkController.setInverted(InvertType.None);
        _rightPivotLinkController.setInverted(InvertType.InvertMotorOutput);

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
            double LPos = _leftclimberController.getSelectedSensorPosition();
            double RPos = _rightclimberController.getSelectedSensorPosition();
            boolean isLDone = (Math.abs(LPos - _lastLClimberHeight) < 5.0);
            boolean isRDone = (Math.abs(RPos - _lastRClimberHeight) < 5.0);

            if (isLDone)
                _leftclimberController.set(0.0);
            if (isRDone)
                _rightclimberController.set(0.0);
            if (isLDone && isRDone) {
                _isResetClimber = false;
                _isClimbing = false;
                smartClimberController.resetPosition();
            }
            _lastLClimberHeight = LPos;
            _lastRClimberHeight = RPos;
        }
        if (_isClimbing) {
            System.out.println("isClimbing - current height = " + smartClimberController.getPosition() + " pos = " + smartClimberController.getNativePosition());
            if (Math.abs(smartClimberController.getPosition() - _targetClimberHeight) < 0.01) {
                _isClimbing = false;
                System.out.println("isClimbing - done");
            }
        }
        if (_isResetPivoting) {
            double LPos = _leftPivotLinkController.getSelectedSensorPosition();
            double RPos = _rightPivotLinkController.getSelectedSensorPosition();
            boolean isLDone = (Math.abs(LPos - _lastLPivotDistance) < 5.0);
            boolean isRDone = (Math.abs(RPos - _lastRPivotDistance) < 5.0);

            if (isLDone)
            _leftPivotLinkController.set(0.0);
            if (isRDone)
            _rightPivotLinkController.set(0.0);
            if (isLDone && isRDone) {
                _isResetPivoting = false;
                _isPivoting = false;
                smartPivotLinkController.resetPosition();
            }
            _lastLPivotDistance = LPos;
            _lastRPivotDistance = RPos;
        }
        if (_isPivoting) {
            System.out.println("isPivoting - current distance = " + smartPivotLinkController.getPosition() + smartPivotLinkController.getNativePosition());
            if (Math.abs(smartPivotLinkController.getPosition() - _targetPivotDistance) < 0.01) {
                _isPivoting = false;
                System.out.println("isPivoting - done");
            }
        }
        smartClimberController.update();
        smartPivotLinkController.update();
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void logPeriodic() {
        SmartDashboard.putBoolean("isResetClimber", _isResetClimber);
        SmartDashboard.putBoolean("isResetPivoting", _isResetPivoting);
        SmartDashboard.putBoolean("isClimbing", _isClimbing);
        SmartDashboard.putBoolean("isPivoting", _isPivoting);
        SmartDashboard.putNumber("targetHeight", _targetClimberHeight);
        SmartDashboard.putNumber("targetPivot", _targetPivotDistance);

        SmartDashboard.putNumber("ClimberPos", smartClimberController.getNativePosition());
        SmartDashboard.putNumber("PivotPos", smartPivotLinkController.getNativePosition());
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void runClimber(double height) {
        _targetClimberHeight = height;

        smartClimberController.setTarget(_targetClimberHeight); //, _climberFeedFwd);
        _isClimbing = true;

        // System.out.println("isClimbing - target (meters) = " + _targetHeight);
    }
    public boolean isClimbing() { return _isClimbing; }
    public void setClimber(double speed) {
        if (!_isClimbing && !_isResetClimber) {
            smartClimberController.set(speed);
        }
    }

    public void resetClimber() {
        _lastRClimberHeight = smartClimberController.getNativePosition();
        _isResetClimber = true;
        _leftclimberController.set(kResetClimberSpeed);
        _rightclimberController.set(kResetClimberSpeed);
    }

    public void runPivotLink(double distance) {
        _targetPivotDistance = distance;

        smartPivotLinkController.setTarget(_targetPivotDistance); //, _climberFeedFwd);
        _isPivoting = true;

        // System.out.println("isPivoting - target (meters) = " + _targetPivot);
    }
    public boolean isPivoting() { return _isPivoting; }
    public void setPivotLink(double speed) {
        if (!_isPivoting && !_isResetPivoting) {
            smartPivotLinkController.set(speed);
        }
    }

    public void resetPivotLink() {
        _isResetPivoting = true;
        _lastRPivotDistance = smartPivotLinkController.getNativePosition();
        smartPivotLinkController.set(kResetPivotSpeed);
    }
    public boolean isResetting() { return _isResetClimber || _isResetPivoting; }
}

