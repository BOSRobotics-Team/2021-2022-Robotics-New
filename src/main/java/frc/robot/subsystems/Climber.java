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

  private final SmartMotorController smartLClimberController =
      new SmartMotorController(_leftClimberController, "LClimber");
  private final SmartMotorController smartRClimberController =
      new SmartMotorController(_rightClimberController, "RClimber");
  private final SmartMotorController smartLPivotLinkController =
      new SmartMotorController(_leftPivotLinkController, "LPivot");
  private final SmartMotorController smartRPivotLinkController =
      new SmartMotorController(_rightPivotLinkController, "RPivot");

  private final double kResetClimberSpeed = -0.05;
  private boolean _isClimbing = false;
  private boolean _isResetLClimber = false;
  private boolean _isResetRClimber = false;
  private boolean _isClimberRevLimitSwitchTest = false;
  private double _targetClimberHeight = 0;
  private double _climberMaxHeight = 0.55;
  private double _climberFeedFwd = 0.0;

  private final double kPivotLinkMass = 3.0; // Kilograms
  private final double kPivotLinkLength = Units.inchesToMeters(30);
  private final double kResetPivotSpeed = 0.025;
  private boolean _isPivoting = false;
  private boolean _isResetLPivoting = false;
  private boolean _isResetRPivoting = false;
  private boolean _isPivotRevLimitSwitchTest = false;
  private double _targetPivotAngle = 0;
  private double _minPivotLinkAngle = 40.0;
  private double _maxPivotLinkAngle = 100.0;
  private double _pivotFeedFwd = 0.0;

  private final ClimberSim _climberSim;

  public Climber() {
    Preferences.initDouble("ClimberMaxHeight", 0.55);
    Preferences.initDouble("ClimberFeedForward", 0.0);
    Preferences.initDouble("PivotFeedForward", 0.0);

    _climberMaxHeight = Preferences.getDouble("ClimberMaxHeight", 0.55);
    _climberFeedFwd = Preferences.getDouble("ClimberFeedForward", 0.0);
    _pivotFeedFwd = Preferences.getDouble("PivotFeedForward", 0.0);

    _leftClimberController.setInverted(InvertType.None);
    smartLClimberController.initController();
    smartLClimberController.configureRatios(Constants.kClimberGearRatio);
    smartLClimberController.enableBrakes(true);
    smartLClimberController.setDistanceConfigs(Constants.kClimberGains_Distance);

    _rightClimberController.setInverted(InvertType.InvertMotorOutput);
    smartRClimberController.initController();
    smartRClimberController.configureRatios(Constants.kClimberGearRatio);
    smartRClimberController.enableBrakes(true);
    smartRClimberController.setDistanceConfigs(Constants.kClimberGains_Distance);

    _leftPivotLinkController.setInverted(InvertType.None);
    smartLPivotLinkController.initController();
    smartLPivotLinkController.configureRatios(Constants.kPivotLinkGearRatio);
    smartLPivotLinkController.enableBrakes(true);
    smartLPivotLinkController.setDistanceConfigs(Constants.kPivotLinkGains_Distance);

    _rightPivotLinkController.setInverted(InvertType.InvertMotorOutput);
    smartRPivotLinkController.initController();
    smartRPivotLinkController.configureRatios(Constants.kPivotLinkGearRatio);
    smartRPivotLinkController.enableBrakes(true);
    smartRPivotLinkController.setDistanceConfigs(Constants.kPivotLinkGains_Distance);

    _climberSim =
        new ClimberSim(
            _leftClimberController,
            _rightClimberController,
            smartLClimberController.convertor,
            Constants.kDriveCarriageMass,
            Units.inchesToMeters(21.65),
            _leftPivotLinkController,
            _rightPivotLinkController,
            smartLPivotLinkController.convertor,
            kPivotLinkMass,
            kPivotLinkLength,
            _minPivotLinkAngle,
            _maxPivotLinkAngle);

    this.zeroClimberPosition();
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if (_isResetLClimber) {
      if (isLClimberRevLimitSwitchClosed()) {
        smartLClimberController.set(0.0);
        smartLClimberController.resetPosition();
        _isResetLClimber = false;
        System.out.println("isResetLClimber - done");
      }
    }
    if (_isResetRClimber) {
      if (isRClimberRevLimitSwitchClosed()) {
        smartRClimberController.set(0.0);
        smartRClimberController.resetPosition();
        _isResetRClimber = false;
        System.out.println("isResetRClimber - done");
      }
    }
    if (_isResetLPivoting) {
      if (isLPivotFwdLimitSwitchClosed()) {
        smartLPivotLinkController.set(0.0);
        smartLPivotLinkController.resetPosition();
        _isResetLPivoting = false;
        System.out.println("isResetLPivot - done");
      }
    }
    if (_isResetRPivoting) {
      if (isRPivotFwdLimitSwitchClosed()) {
        smartRPivotLinkController.set(0.0);
        smartRPivotLinkController.resetPosition();
        _isResetRPivoting = false;
        System.out.println("isResetRPivot - done");
      }
    }
    if (_isClimbing) {
      System.out.println(
          "isClimbingLeft - current height = "
              + smartLClimberController.getDistance()
              + " pos = "
              + smartLClimberController.getPosition());
      System.out.println(
          "isClimbingRight - current height = "
              + smartRClimberController.getDistance()
              + " pos = "
              + smartRClimberController.getPosition());
      if (smartLClimberController.isTargetFinished()
          && smartRClimberController.isTargetFinished()) {
        _isClimbing = false;
        System.out.println("isClimbing - done");
      }
    }
    if (_isPivoting) {
      System.out.println(
          "isPivotingLeft - current distance = "
              + smartLPivotLinkController.getDistance()
              + " pos = "
              + smartLPivotLinkController.getPosition());
      System.out.println(
          "isPivotingRight - current distance = "
              + smartRPivotLinkController.getDistance()
              + " pos = "
              + smartRPivotLinkController.getPosition());
      if (smartLPivotLinkController.isTargetFinished()
          && smartRPivotLinkController.isTargetFinished()) {
        _isPivoting = false;
        System.out.println("isPivoting - done");
      }
    }
    smartLClimberController.update();
    smartRClimberController.update();
    smartLPivotLinkController.update();
    smartRPivotLinkController.update();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    _climberSim.run();
  }

  public void logPeriodic() {
    smartLClimberController.logPeriodic();
    smartRClimberController.logPeriodic();
    smartLPivotLinkController.logPeriodic();
    smartRPivotLinkController.logPeriodic();
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void setClimberHeight(double height) {
    _targetClimberHeight = MathUtil.clamp(height, 0.0, _climberMaxHeight);
    smartLClimberController.setTarget(_targetClimberHeight, _climberFeedFwd);
    smartRClimberController.setTarget(_targetClimberHeight, _climberFeedFwd);
    _isClimbing = true;
  }

  public void setClimberHeightPct(double pctHeight) {
    setClimberHeight(pctHeight * _climberMaxHeight);
  }

  public void setClimberHeightInc(double pctInc) {
    setClimberHeight(_targetClimberHeight + pctInc);
  }

  public double getClimberHeight() {
    return smartLClimberController.getDistance();
    // return smartRClimberController.getDistance();
  }

  public double getClimberHeightPct() {
    return getClimberHeight() / _climberMaxHeight;
  }

  public boolean isClimbing() {
    return _isClimbing;
  }

  public boolean isResettingClimber() {
    return _isResetLClimber || _isResetRClimber;
  }

  public void setClimberSpeed(double speedL, double speedR) {
    if (!isResettingClimber()) {
      smartLClimberController.set(speedL);
      smartRClimberController.set(speedR);
      _isClimbing = false;
    }
  }

  public void setClimberSpeed(double speedL, double speedR, double arbFF) {
    smartLClimberController.configureFeedForward(arbFF);
    smartRClimberController.configureFeedForward(arbFF);
    setClimberSpeed(speedL, speedR);
  }

  public void resetClimber() {
    _isClimbing = false;
    _isResetLClimber = _isResetRClimber = true;
    double lSpeed = isLClimberRevLimitSwitchClosed() ? 0.0 : kResetClimberSpeed;
    smartLClimberController.set(lSpeed);
    double rSpeed = isRClimberRevLimitSwitchClosed() ? 0.0 : kResetClimberSpeed;
    smartRClimberController.set(rSpeed);

    this.resetPivotLink();
  }

  public void zeroClimberPosition() {
    smartLClimberController.resetPosition();
    smartRClimberController.resetPosition();
    smartLPivotLinkController.resetPosition();
    smartRPivotLinkController.resetPosition();
  }

  public void setPivotLinkAngle(double angleDegrees) {
    _targetPivotAngle = MathUtil.clamp(angleDegrees, _minPivotLinkAngle, _maxPivotLinkAngle);
    System.out.println(
        "setPivotLinkAngle: "
            + angleDegrees
            + " tgtAngle: "
            + _targetPivotAngle
            + " setTgt: "
            + (_targetPivotAngle - _maxPivotLinkAngle));
    smartLPivotLinkController.setTarget(_targetPivotAngle - _maxPivotLinkAngle, _pivotFeedFwd);
    smartRPivotLinkController.setTarget(_targetPivotAngle - _maxPivotLinkAngle, _pivotFeedFwd);
    _isPivoting = true;
  }

  public void setPivotLinkAnglePct(double pctAngle) {
    setPivotLinkAngle(pctAngle * (_maxPivotLinkAngle - _minPivotLinkAngle) + _minPivotLinkAngle);
  }

  public void setPivotLinkAngleInc(double pctInc) {
    setPivotLinkAngle(_targetPivotAngle + pctInc);
  }

  public double getPivotLinkAngle() {
    return smartLPivotLinkController.getDistance();
    // return smartRPivotLinkController.getDistance();
  }

  public double getPivotLinkAnglePct() {
    return getPivotLinkAngle() / (_maxPivotLinkAngle - _minPivotLinkAngle);
  }

  public boolean isPivoting() {
    return _isPivoting;
  }

  public boolean isResettingPivot() {
    return _isResetLPivoting || _isResetRPivoting;
  }

  public void setPivotLinkSpeed(double speedL, double speedR) {
    if (!isResettingPivot()) {
      smartLPivotLinkController.set(speedL);
      smartRPivotLinkController.set(speedR);
      _isPivoting = false;
    }
  }

  public void setPivotLinkSpeed(double speedL, double speedR, double arbFF) {
    smartLPivotLinkController.configureFeedForward(arbFF);
    smartRPivotLinkController.configureFeedForward(arbFF);
    setPivotLinkSpeed(speedL, speedR);
  }

  public void resetPivotLink() {
    _isPivoting = false;
    _isResetLPivoting = _isResetRPivoting = true;
    double lSpeed = isLPivotFwdLimitSwitchClosed() ? 0.0 : kResetPivotSpeed;
    smartLPivotLinkController.set(lSpeed);

    double rSpeed = isRPivotFwdLimitSwitchClosed() ? 0.0 : kResetPivotSpeed;
    smartRPivotLinkController.set(rSpeed);
  }

  public boolean isResetting() {
    return isResettingClimber() || isResettingPivot();
  }

  public boolean isLClimberRevLimitSwitchClosed() {
    return smartLClimberController.isPrimaryFwdLimitSwitchClosed() || _isClimberRevLimitSwitchTest;
  }

  public boolean isLClimberFwdLimitSwitchClosed() {
    return smartLClimberController.isPrimaryFwdLimitSwitchClosed() || _isClimberRevLimitSwitchTest;
  }

  public boolean isRClimberRevLimitSwitchClosed() {
    return smartRClimberController.isPrimaryRevLimitSwitchClosed() || _isClimberRevLimitSwitchTest;
  }

  public boolean isRClimberFwdLimitSwitchClosed() {
    return smartRClimberController.isPrimaryFwdLimitSwitchClosed() || _isClimberRevLimitSwitchTest;
  }

  public boolean isLPivotRevLimitSwitchClosed() {
    return smartLPivotLinkController.isPrimaryFwdLimitSwitchClosed() || _isPivotRevLimitSwitchTest;
  }

  public boolean isLPivotFwdLimitSwitchClosed() {
    return smartLPivotLinkController.isPrimaryFwdLimitSwitchClosed() || _isPivotRevLimitSwitchTest;
  }

  public boolean isRPivotRevLimitSwitchClosed() {
    return smartRPivotLinkController.isPrimaryRevLimitSwitchClosed() || _isPivotRevLimitSwitchTest;
  }

  public boolean isRPivotFwdLimitSwitchClosed() {
    return smartRPivotLinkController.isPrimaryFwdLimitSwitchClosed() || _isPivotRevLimitSwitchTest;
  }

  public void tripClimberRevLimitSwitches_test(boolean trip) {
    _isClimberRevLimitSwitchTest = trip;
  }

  public void tripPivotRevLimitSwitches_test(boolean trip) {
    _isPivotRevLimitSwitchTest = trip;
  }
}
