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
    _rightClimberController.setInverted(InvertType.InvertMotorOutput);

    smartClimberController.initController();
    smartClimberController.configureRatios(Constants.kClimberGearRatio);
    smartClimberController.enableBrakes(true);
    smartClimberController.setDistanceAndTurnConfigs(
        Constants.kClimberGains_Distance, Constants.kClimberGains_Turn);

    _leftPivotLinkController.setInverted(InvertType.None);
    _rightPivotLinkController.setInverted(InvertType.InvertMotorOutput);

    smartPivotLinkController.initController();
    smartPivotLinkController.configureRatios(Constants.kPivotLinkGearRatio);
    smartPivotLinkController.enableBrakes(true);
    smartPivotLinkController.setDistanceAndTurnConfigs(
        Constants.kPivotLinkGains_Distance, Constants.kPivotLinkGains_Turn);

    _climberSim =
        new ClimberSim(
            _leftClimberController,
            _rightClimberController,
            smartClimberController.convertor,
            Constants.kDriveCarriageMass,
            Units.inchesToMeters(21.65),
            _leftPivotLinkController,
            _rightPivotLinkController,
            smartPivotLinkController.convertor,
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
        _leftClimberController.set(0.0);
        smartClimberController.resetLeftPosition();
        _isResetLClimber = false;
        System.out.println("isResetLClimber - done");
      }
    }
    if (_isResetRClimber) {
      if (isRClimberRevLimitSwitchClosed()) {
        _rightClimberController.set(0.0);
        smartClimberController.resetRightPosition();
        _isResetRClimber = false;
        System.out.println("isResetRClimber - done");
      }
    }
    if (_isResetLPivoting) {
      if (isLPivotFwdLimitSwitchClosed()) {
        _leftPivotLinkController.set(0.0);
        smartPivotLinkController.resetLeftPosition();
        _isResetLPivoting = false;
        System.out.println("isResetLPivot - done");
      }
    }
    if (_isResetRPivoting) {
      if (isRPivotFwdLimitSwitchClosed()) {
        _rightPivotLinkController.set(0.0);
        smartPivotLinkController.resetRightPosition();
        _isResetRPivoting = false;
        System.out.println("isResetRPivot - done");
      }
    }
    if (_isClimbing) {
      System.out.println(
          "isClimbing - current height = "
              + smartClimberController.getDistance()
              + " pos = "
              + smartClimberController.getPosition());
      if (smartClimberController.isTargetFinished()) {
        _isClimbing = false;
        System.out.println("isClimbing - done");
      }
    }
    if (_isPivoting) {
      System.out.println(
          "isPivoting - current distance = "
              + smartPivotLinkController.getDistance()
              + smartPivotLinkController.getPosition());
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
    _climberSim.run();
  }

  public void logPeriodic() {
    smartClimberController.logPeriodic();
    smartPivotLinkController.logPeriodic();
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void setClimberHeight(double height) {
    _targetClimberHeight = MathUtil.clamp(height, 0.0, _climberMaxHeight);
    if (_climberFeedFwd > 0)
      smartClimberController.setTarget(_targetClimberHeight, _climberFeedFwd);
    else smartClimberController.setTargetAndAngle(_targetClimberHeight, 0);
    _isClimbing = true;
  }

  public void setClimberHeightPct(double pctHeight) {
    setClimberHeight(pctHeight * _climberMaxHeight);
  }

  public void setClimberHeightInc(double pctInc) {
    setClimberHeight(_targetClimberHeight + pctInc);
  }

  public double getClimberHeight() {
    return smartClimberController.getDistance();
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
      smartClimberController.set(speedR, speedL);
      _isClimbing = false;
    }
  }

  public void setClimberSpeed(double speedL, double speedR, double arbFF) {
    smartClimberController.configureFeedForward(arbFF);
    setClimberSpeed(speedL, speedR);
  }

  public void resetClimber() {
    _isClimbing = false;
    _isResetLClimber = _isResetRClimber = true;
    double rSpeed = isRClimberRevLimitSwitchClosed() ? 0.0 : kResetClimberSpeed;
    double lSpeed = isLClimberRevLimitSwitchClosed() ? 0.0 : kResetClimberSpeed;
    smartClimberController.set(rSpeed, lSpeed);

    this.resetPivotLink();
  }

  public void zeroClimberPosition() {
    smartClimberController.resetPosition();
    smartPivotLinkController.resetPosition();
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
    // smartPivotLinkController.setTargetAndAngle(_targetPivotAngle - _maxPivotLinkAngle, 0);
    smartPivotLinkController.setTarget(_targetPivotAngle - _maxPivotLinkAngle, _pivotFeedFwd);
    _isPivoting = true;
  }

  public void setPivotLinkAnglePct(double pctAngle) {
    setPivotLinkAngle(pctAngle * (_maxPivotLinkAngle - _minPivotLinkAngle) + _minPivotLinkAngle);
  }

  public void setPivotLinkAngleInc(double pctInc) {
    setPivotLinkAngle(_targetPivotAngle + pctInc);
  }

  public double getPivotLinkAngle() {
    return smartPivotLinkController.getDistance();
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
      smartPivotLinkController.set(speedR, speedL);
      _isPivoting = false;
    }
  }

  public void setPivotLinkSpeed(double speedL, double speedR, double arbFF) {
    smartPivotLinkController.configureFeedForward(arbFF);
    setPivotLinkSpeed(speedL, speedR);
  }

  public void resetPivotLink() {
    _isPivoting = false;
    _isResetLPivoting = _isResetRPivoting = true;
    double rSpeed = isRPivotFwdLimitSwitchClosed() ? 0.0 : kResetPivotSpeed;
    double lSpeed = isLPivotFwdLimitSwitchClosed() ? 0.0 : kResetPivotSpeed;
    smartPivotLinkController.set(rSpeed, lSpeed);
  }

  public boolean isResetting() {
    return isResettingClimber() || isResettingPivot();
  }

  public boolean isLClimberRevLimitSwitchClosed() {
    return smartClimberController.isLeftRevLimitSwitchClosed() || _isClimberRevLimitSwitchTest;
  }

  public boolean isLClimberFwdLimitSwitchClosed() {
    return smartClimberController.isLeftFwdLimitSwitchClosed() || _isClimberRevLimitSwitchTest;
  }

  public boolean isRClimberRevLimitSwitchClosed() {
    return smartClimberController.isRightRevLimitSwitchClosed() || _isClimberRevLimitSwitchTest;
  }

  public boolean isRClimberFwdLimitSwitchClosed() {
    return smartClimberController.isRightFwdLimitSwitchClosed() || _isClimberRevLimitSwitchTest;
  }

  public boolean isLPivotRevLimitSwitchClosed() {
    return smartPivotLinkController.isLeftRevLimitSwitchClosed() || _isPivotRevLimitSwitchTest;
  }

  public boolean isLPivotFwdLimitSwitchClosed() {
    return smartPivotLinkController.isLeftFwdLimitSwitchClosed() || _isPivotRevLimitSwitchTest;
  }

  public boolean isRPivotRevLimitSwitchClosed() {
    return smartPivotLinkController.isRightRevLimitSwitchClosed() || _isPivotRevLimitSwitchTest;
  }

  public boolean isRPivotFwdLimitSwitchClosed() {
    return smartPivotLinkController.isRightFwdLimitSwitchClosed() || _isPivotRevLimitSwitchTest;
  }

  public void tripClimberRevLimitSwitches_test(boolean trip) {
    _isClimberRevLimitSwitchTest = trip;
  }

  public void tripPivotRevLimitSwitches_test(boolean trip) {
    _isPivotRevLimitSwitchTest = trip;
  }
}
