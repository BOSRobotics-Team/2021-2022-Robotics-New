// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sim.ClimberSim;
import frc.robot.util.*;

public class Climber extends SubsystemBase {
  private final WPI_TalonFX _leftClimberController = new WPI_TalonFX(Constants.kID_LClimber);
  private final WPI_TalonFX _rightClimberController = new WPI_TalonFX(Constants.kID_RClimber);
  private final WPI_TalonFX _leftPivotLinkController = new WPI_TalonFX(Constants.kID_LPivot);
  private final WPI_TalonFX _rightPivotLinkController = new WPI_TalonFX(Constants.kID_RPivot);

  private final SmartMotorController smartClimberController =
      new SmartMotorController(_rightClimberController, _leftClimberController, "Climber");
  private final SmartMotorController smartPivotLinkController =
      new SmartMotorController(_rightPivotLinkController, _leftPivotLinkController, "Pivot");

  private final GearRatios kGearRatio_Climber = new GearRatios(20.0, 0.5, 1.0);
  private final Gains kGains_Climber = new Gains(0.2, 0.0, 0.0, 0., 0, 1.0);
  private final Gains kGains_ClimberTurn = new Gains(0.1, 0.0, 0.0, 0., 0, 1.0);
  private final double kCarriageMass = 54.0; // Kilograms
  private final double kResetClimberSpeed = -0.05;
  private boolean _isClimbing = false;
  private boolean _isResetLClimber = false;
  private boolean _isResetRClimber = false;
  private double _currLClimberHeight = 0.0;
  private double _currRClimberHeight = 0.0;
  private double _lastLClimberHeight = 0.0;
  private double _lastRClimberHeight = 0.0;
  private double _targetClimberHeight = 0;
  private double _climberMaxHeight = 1.5;
  // private double _climberFeedFwd = 0.1;

  private final GearRatios kGearRatio_PivotLink =
      new GearRatios(100.0, Convertor.kWheelRadiusForDegrees, 2.0);
  private final Gains kGains_PivotLink = new Gains(0.2, 0.0, 0.0, 0.2, 0, 0.3);
  private final Gains kGains_PivotLinkTurn = new Gains(0.1, 0.0, 0.0, 0.1, 0, 0.3);
  private final double kPivotLinkMass = 3.0; // Kilograms
  private final double kPivotLinkLength = Units.inchesToMeters(30);
  private final double kMinPivotLinkAngle = 50.0;
  private final double kMaxPivotLinkAngle = 130.0;
  private final double kResetPivotSpeed = -0.05;
  private boolean _isPivoting = false;
  private boolean _isResetLPivoting = false;
  private boolean _isResetRPivoting = false;
  private double _currLPivotAngle = 0.0;
  private double _currRPivotAngle = 0.0;
  private double _lastLPivotAngle = 0.0;
  private double _lastRPivotAngle = 0.0;
  private double _targetPivotAngle = 0;
  private double _resetThresholdUnits = 10;

  private final ClimberSim _climberSim =
      new ClimberSim(
          _leftClimberController,
          _rightClimberController,
          kGearRatio_Climber,
          kCarriageMass,
          Units.inchesToMeters(21.65),
          _leftPivotLinkController,
          _rightPivotLinkController,
          kGearRatio_PivotLink,
          kPivotLinkMass,
          kPivotLinkLength,
          kMinPivotLinkAngle,
          kMaxPivotLinkAngle);

  public Climber() {
    Preferences.initDouble("ClimberMaxHeight", 0.55);
    Preferences.initDouble("ResetThresholdUnits", 10.0);

    _climberMaxHeight = Preferences.getDouble("ClimberMaxHeight", 0.55);
    _resetThresholdUnits = Preferences.getDouble("ResetThresholdUnits", 10.0);

    _leftClimberController.setInverted(InvertType.None);
    _rightClimberController.setInverted(InvertType.InvertMotorOutput);

    smartClimberController.initController();
    smartClimberController.configureRatios(kGearRatio_Climber);
    smartClimberController.enableBrakes(true);
    smartClimberController.setDistanceAndTurnConfigs(kGains_Climber, kGains_ClimberTurn);
    smartClimberController.resetPosition();

    _leftPivotLinkController.setInverted(InvertType.None);
    _rightPivotLinkController.setInverted(InvertType.InvertMotorOutput);

    smartPivotLinkController.initController();
    smartPivotLinkController.configureRatios(kGearRatio_PivotLink);
    smartPivotLinkController.enableBrakes(true);
    smartPivotLinkController.setDistanceAndTurnConfigs(kGains_PivotLink, kGains_PivotLinkTurn);
    smartPivotLinkController.resetPosition();
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if (_isResetLClimber) {
      _currLClimberHeight = smartClimberController.getLeftPosition();
      if (Math.abs(_currLClimberHeight - _lastLClimberHeight) < _resetThresholdUnits) {
        _leftClimberController.set(0.0);
        smartClimberController.resetLeftPosition();
        _isResetLClimber = false;
      }
      _lastLClimberHeight = _currLClimberHeight;
    }
    if (_isResetRClimber) {
      _currRClimberHeight = smartClimberController.getRightPosition();
      if (Math.abs(_currRClimberHeight - _lastRClimberHeight) < _resetThresholdUnits) {
        _rightClimberController.set(0.0);
        smartClimberController.resetRightPosition();
        _isResetRClimber = false;
      }
      _lastRClimberHeight = _currRClimberHeight;
    }
    if (_isResetLPivoting) {
      _currLPivotAngle = smartPivotLinkController.getLeftPosition();
      if (Math.abs(_currLPivotAngle - _lastLPivotAngle) < _resetThresholdUnits) {
        _leftPivotLinkController.set(0.0);
        smartPivotLinkController.resetLeftPosition();
        _isResetLPivoting = false;
      }
      _lastLPivotAngle = _currLPivotAngle;
    }
    if (_isResetRPivoting) {
      _currRPivotAngle = smartPivotLinkController.getRightPosition();
      if (Math.abs(_currRPivotAngle - _lastRPivotAngle) < _resetThresholdUnits) {
        _rightPivotLinkController.set(0.0);
        smartPivotLinkController.resetRightPosition();
        _isResetRPivoting = false;
      }
      _lastRPivotAngle = _currRPivotAngle;
    }
    if (_isClimbing) {
      // System.out.println(
      //     "isClimbing - current height = "
      //         + smartClimberController.getDistance()
      //         + " pos = "
      //         + smartClimberController.getPosition());
      if (smartClimberController.isTargetFinished()) {
        _isClimbing = false;
        // System.out.println("isClimbing - done");
      }
    }
    if (_isPivoting) {
      // System.out.println(
      //     "isPivoting - current distance = "
      //         + smartPivotLinkController.getDistance()
      //         + smartPivotLinkController.getPosition());
      if (smartPivotLinkController.isTargetFinished()) {
        _isPivoting = false;
        // System.out.println("isPivoting - done");
      }
    }
    smartClimberController.update();
    smartPivotLinkController.update();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    _climberSim.run();
  }

  public void logPeriodic() {
    smartClimberController.logPeriodic();
    smartPivotLinkController.logPeriodic();
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void setClimberHeight(double pctHeight) {
    _targetClimberHeight = MathUtil.clamp(pctHeight, 0.0, 1.0) * _climberMaxHeight;
    smartClimberController.setTarget(_targetClimberHeight, 0); // , _climberFeedFwd);
    _isClimbing = true;
  }

  public double getClimberHeight() {
    return smartClimberController.getDistance() / _climberMaxHeight;
  }

  public boolean isClimbing() {
    return _isClimbing;
  }
  public boolean isResettingClimber() {
    return _isResetLClimber || _isResetRClimber;
  }

  public void setClimberSpeed(double speed) {
    if (!isResettingClimber()) {
      smartClimberController.set(speed);
      _isClimbing = false;
    }
  }

  public void resetClimber() {
    _isClimbing = false;
    _isResetLClimber = _isResetRClimber = true;
    _lastLClimberHeight = smartClimberController.getLeftPosition();
    _lastRClimberHeight = smartClimberController.getRightPosition();
    smartClimberController.set(kResetClimberSpeed, kResetClimberSpeed);
  }

  public void setPivotLinkAngle(double pctAngle) {
    _targetPivotAngle =
        MathUtil.clamp(pctAngle, 0.0, 1.0) * (kMaxPivotLinkAngle - kMinPivotLinkAngle);
    smartPivotLinkController.setTarget(_targetPivotAngle, 0); // , _climberFeedFwd);
    _isPivoting = true;
  }

  public double getPivotLinkAngle() {
    return smartPivotLinkController.getDistance() / (kMaxPivotLinkAngle - kMinPivotLinkAngle);
  }

  public boolean isPivoting() {
    return _isPivoting;
  }
  public boolean isResettingPivot() {
    return _isResetLPivoting || _isResetRPivoting;
  }

  public void setPivotLinkSpeed(double speed) {
    if (!isResettingPivot()) {
      smartPivotLinkController.set(speed);
      _isPivoting = false;
    }
  }

  public void resetPivotLink() {
    _isPivoting = false;
    _isResetLPivoting = _isResetRPivoting = true;
    _lastLPivotAngle = smartPivotLinkController.getLeftPosition();
    _lastRPivotAngle = smartPivotLinkController.getRightPosition();
    smartPivotLinkController.set(kResetPivotSpeed, kResetPivotSpeed);
  }

  public boolean isResetting() {
    return isResettingClimber() || isResettingPivot();
  }
}
