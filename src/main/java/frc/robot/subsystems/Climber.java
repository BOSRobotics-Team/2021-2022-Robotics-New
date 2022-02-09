// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.*;

public class Climber extends SubsystemBase {
  private final WPI_TalonFX _leftclimberController = new WPI_TalonFX(9);
  private final WPI_TalonFX _rightclimberController = new WPI_TalonFX(10);
  private final WPI_TalonFX _leftPivotLinkController = new WPI_TalonFX(11);
  private final WPI_TalonFX _rightPivotLinkController = new WPI_TalonFX(12);

  private final SmartMotorController smartClimberController =
      new SmartMotorController(_rightclimberController, _leftclimberController);
  private final SmartMotorController smartPivotLinkController =
      new SmartMotorController(_rightPivotLinkController, _leftPivotLinkController);

  private final GearRatios kGearRatio_Climber = new GearRatios(12.0, 1.0, 1.0);
  public final Gains kGains_Climber = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.80);
  private boolean _isClimbing = false;
  private boolean _isResetClimber = false;
  private double _lastLClimberHeight = 0.0;
  private double _lastRClimberHeight = 0.0;
  private double _targetClimberHeight = 0;
  private double kResetClimberSpeed = -0.1;
  private double _climberMaxHeight = 1.5;
  // private double _climberFeedFwd = 0.1;

  private final GearRatios kGearRatio_PivotLink = new GearRatios(20.0, 1.0, 1.0);
  public final Gains kGains_PivotLink = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.80);
  private boolean _isPivoting = false;
  private boolean _isResetPivoting = false;
  private double _lastLPivotDistance = 0.0;
  private double _lastRPivotDistance = 0.0;
  private double _targetPivotDistance = 0;
  private double kResetPivotSpeed = -0.05;
  private double _pivotMaxDistance = 1.0;

  public Climber() {
    Preferences.initDouble("ClimberMaxHeight", 1.1);
    Preferences.initDouble("PivotMaxDistance", 1.0);

    // Preferences.initDouble("ClimberFeedFwd", 0.5);
    // _climberFeedFwd = Preferences.getDouble("ClimberFeedFwd", 0.1);
    _climberMaxHeight = Preferences.getDouble("ClimberMaxHeight", 1.1);
    _pivotMaxDistance = Preferences.getDouble("PivotMaxDistance", 1.0);

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

      if (isLDone) _leftclimberController.set(0.0);
      if (isRDone) _rightclimberController.set(0.0);
      if (isLDone && isRDone) {
        _isResetClimber = false;
        _isClimbing = false;
        smartClimberController.resetPosition();
      }
      _lastLClimberHeight = LPos;
      _lastRClimberHeight = RPos;
    }
    if (_isClimbing) {
      System.out.println(
          "isClimbing - current height = "
              + smartClimberController.getPosition()
              + " pos = "
              + smartClimberController.getNativePosition());
      if (smartClimberController.isTargetFinished()) {
        _isClimbing = false;
        System.out.println("isClimbing - done");
      }
    }
    if (_isResetPivoting) {
      double LPos = _leftPivotLinkController.getSelectedSensorPosition();
      double RPos = _rightPivotLinkController.getSelectedSensorPosition();
      boolean isLDone = (Math.abs(LPos - _lastLPivotDistance) < 5.0);
      boolean isRDone = (Math.abs(RPos - _lastRPivotDistance) < 5.0);

      if (isLDone) _leftPivotLinkController.set(0.0);
      if (isRDone) _rightPivotLinkController.set(0.0);
      if (isLDone && isRDone) {
        _isResetPivoting = false;
        _isPivoting = false;
        smartPivotLinkController.resetPosition();
      }
      _lastLPivotDistance = LPos;
      _lastRPivotDistance = RPos;
    }
    if (_isPivoting) {
      System.out.println(
          "isPivoting - current distance = "
              + smartPivotLinkController.getPosition()
              + smartPivotLinkController.getNativePosition());
      if (smartPivotLinkController.isTargetFinished()) {
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
  public void setClimberHeight(double pctHeight) {
    _targetClimberHeight = MathUtil.clamp(pctHeight, 0.0, 1.0) * _climberMaxHeight;
    smartClimberController.setTarget(_targetClimberHeight); // , _climberFeedFwd);
    _isClimbing = true;
  }

  public double getClimberHeight() {
    return smartClimberController.getPosition() / _climberMaxHeight;
  }

  public boolean isClimbing() {
    return _isClimbing;
  }

  public void setClimberSpeed(double speed) {
    if (!_isResetClimber) {
      smartClimberController.set(speed);
      _isClimbing = false;
    }
  }

  public void resetClimber() {
    _isResetClimber = true;
    _isClimbing = false;
    _lastLClimberHeight = _leftclimberController.getSelectedSensorPosition();
    _lastRClimberHeight = _rightclimberController.getSelectedSensorPosition();
    _leftclimberController.set(kResetClimberSpeed);
    _rightclimberController.set(kResetClimberSpeed);
  }

  public void setPivotLinkDistance(double pctDistance) {
    _targetPivotDistance = MathUtil.clamp(pctDistance, 0.0, 1.0) * _pivotMaxDistance;
    smartPivotLinkController.setTarget(_targetPivotDistance); // , _climberFeedFwd);
    _isPivoting = true;
  }

  public double getPivotLinkDistance() {
    return smartPivotLinkController.getPosition() / _pivotMaxDistance;
  }

  public boolean isPivoting() {
    return _isPivoting;
  }

  public void setPivotLinkSpeed(double speed) {
    if (!_isResetPivoting) {
      smartPivotLinkController.set(speed);
      _isPivoting = false;
    }
  }

  public void resetPivotLink() {
    _isPivoting = false;
    _isResetPivoting = true;
    _lastLPivotDistance = _leftPivotLinkController.getSelectedSensorPosition();
    _lastRPivotDistance = _rightPivotLinkController.getSelectedSensorPosition();
    _leftPivotLinkController.set(kResetPivotSpeed);
    _rightPivotLinkController.set(kResetPivotSpeed);
  }

  public boolean isResetting() {
    return _isResetClimber || _isResetPivoting;
  }
}
