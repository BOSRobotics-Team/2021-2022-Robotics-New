// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      new SmartMotorController(_leftClimberController, _rightClimberController, "Climber");
  private final SmartMotorController smartPivotLinkController =
      new SmartMotorController(_leftPivotLinkController, _rightPivotLinkController, "Pivot");

  private int _climbingSeq = -1;

  private boolean _isClimbing = false;
  private boolean _isResetLClimber = false;
  private boolean _isResetRClimber = false;

  private boolean _isClimberLimitSwitchTest = false;
  private double _targetLClimberHeight = 0;
  private double _targetRClimberHeight = 0;
  private double _lClimberMaxHeight = Constants.kLClimberMaxHeight;
  private double _rClimberMaxHeight = Constants.kRClimberMaxHeight;

  private boolean _isPivoting = false;
  private boolean _isResetLPivoting = false;
  private boolean _isResetRPivoting = false;
  private boolean _isPivotLimitSwitchTest = false;
  private double _targetLPivotAngle = 0;
  private double _targetRPivotAngle = 0;
  private double _minPivotLinkAngle = Constants.kPivotLinkMinAngle;
  private double _maxPivotLinkAngle = Constants.kPivotLinkMaxAngle;
  private double _pivotLinkAngleRange = _maxPivotLinkAngle - _minPivotLinkAngle;
  private double _pivotFeedFwd = Constants.kPivotLinkFeedFwd;

  private ClimberSim _climberSim = null;

  public Climber() {
    Preferences.initDouble("ClimberMaxHeight", 0.55);
    _lClimberMaxHeight = Preferences.getDouble("ClimberMaxHeight", 0.55);

    _leftClimberController.setInverted(InvertType.None);
    _rightClimberController.setInverted(InvertType.InvertMotorOutput);
    smartClimberController.initController();
    smartClimberController.configureRatios(Constants.kClimberGearRatio);
    smartClimberController.enableBrakes(true);
    smartClimberController.setSeparateDistanceConfigs(Constants.kClimberGains_Distance);

    _leftPivotLinkController.setInverted(InvertType.None);
    _rightPivotLinkController.setInverted(InvertType.InvertMotorOutput);
    smartPivotLinkController.initController();
    smartPivotLinkController.configureRatios(Constants.kPivotLinkGearRatio);
    smartPivotLinkController.enableBrakes(true);
    smartPivotLinkController.setSeparateDistanceConfigs(Constants.kPivotLinkGains_Distance);

    if (RobotBase.isSimulation()) this.simulationInit();

    this.zeroClimberPosition();
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop

    if (_isResetLClimber && isLClimberRevLimitSwitchClosed()) {
      _isResetLClimber = false;
      _leftClimberController.configClearPositionOnLimitR(false, 0);
      smartClimberController.setOutput(0.0, 0.0);
      _leftClimberController.configForwardSoftLimitThreshold(
          smartClimberController.convertor.distanceMetersToNativeUnits(_lClimberMaxHeight * 1.1));
      _leftClimberController.configForwardSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetLClimber - done: ", EventImportance.kHigh);
      System.out.println("isResetLClimber - done");
    }
    if (_isResetRClimber && isRClimberRevLimitSwitchClosed()) {
      _isResetRClimber = false;
      _rightClimberController.configClearPositionOnLimitR(false, 0);
      smartClimberController.setAuxOutput(0.0, 0.0);
      _rightClimberController.configForwardSoftLimitThreshold(
          smartClimberController.convertor.distanceMetersToNativeUnits(_rClimberMaxHeight * 1.1));
      _rightClimberController.configForwardSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetRClimber - done: ", EventImportance.kHigh);
      System.out.println("isResetRClimber - done");
    }
    if (_isResetLPivoting && isLPivotFwdLimitSwitchClosed()) {
      _isResetLPivoting = false;
      _leftPivotLinkController.configClearPositionOnLimitF(false, 0);
      smartPivotLinkController.setOutput(0.0, 0.0);
      _leftPivotLinkController.configReverseSoftLimitThreshold(
          smartPivotLinkController.convertor.distanceMetersToNativeUnits(
              (_minPivotLinkAngle - _maxPivotLinkAngle) * 1.1));
      _leftPivotLinkController.configReverseSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetLPivot - done: ", EventImportance.kHigh);
      System.out.println("isResetLPivot - done");
    }
    if (_isResetRPivoting && isRPivotFwdLimitSwitchClosed()) {
      _isResetRPivoting = false;
      _rightPivotLinkController.configClearPositionOnLimitF(false, 0);
      smartPivotLinkController.setAuxOutput(0.0, 0.0);
      _rightPivotLinkController.configReverseSoftLimitThreshold(
          smartPivotLinkController.convertor.distanceMetersToNativeUnits(
              (_minPivotLinkAngle - _maxPivotLinkAngle) * 1.1));
      _rightPivotLinkController.configReverseSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetRPivot - done: ", EventImportance.kHigh);
      System.out.println("isResetRPivot - done");
    }
    if (_isClimbing) {
      if (smartClimberController.isTargetFinished()) {
        _isClimbing = false;
        Shuffleboard.addEventMarker("isClimbing - done: ", EventImportance.kHigh);
        System.out.println("isClimbing - done");
      }
    }
    if (_isPivoting) {
      if (smartPivotLinkController.isTargetFinished()) {
        _isPivoting = false;
        Shuffleboard.addEventMarker("isPivoting - done: ", EventImportance.kHigh);
        System.out.println("isPivoting - done");
      }
    }
    smartClimberController.update();
    smartPivotLinkController.update();
  }

  public void simulationInit() {
    _climberSim =
        new ClimberSim(
            _leftClimberController,
            _rightClimberController,
            smartClimberController.convertor,
            0.1, // Constants.kDriveCarriageMass,
            _lClimberMaxHeight,
            _leftPivotLinkController,
            _rightPivotLinkController,
            smartPivotLinkController.convertor,
            Constants.kPivotLinkMass,
            Constants.kPivotLinkLength,
            _minPivotLinkAngle,
            _maxPivotLinkAngle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    _climberSim.run();

    if (isResettingClimber()) tripClimberLimitSwitches_test(true);
    if (isResettingPivot()) tripPivotLimitSwitches_test(true);
    if (isClimbing()) tripClimberLimitSwitches_test(false);
    if (isPivoting()) tripPivotLimitSwitches_test(false);
  }

  public void logPeriodic() {
    smartClimberController.logPeriodic();
    smartPivotLinkController.logPeriodic();
    SmartDashboard.putNumber("ClimbingSequence", _climbingSeq);
    SmartDashboard.putBoolean("isResettingClimber", isResettingClimber());
    SmartDashboard.putBoolean("isResettingPivot", isResettingPivot());
    SmartDashboard.putBoolean("isClimbing", isClimbing());
    SmartDashboard.putBoolean("isPivoting", isPivoting());
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void setClimberHeight(double lHeight, double rHeight, double arbFF) {
    _targetLClimberHeight = lHeight; // MathUtil.clamp(lHeight, -0.025, _lClimberMaxHeight + 0.025);
    _targetRClimberHeight = rHeight; // MathUtil.clamp(rHeight, -0.025, _rClimberMaxHeight + 0.025);

    smartClimberController.setSeparateTarget(_targetLClimberHeight, _targetRClimberHeight, arbFF);
    _isClimbing = true;
    Shuffleboard.addEventMarker("setClimberHeight: ", EventImportance.kHigh);
    System.out.println(
        "setClimberHeight - lHeight: "
            + _targetLClimberHeight
            + " rHeight: "
            + _targetRClimberHeight
            + " arbFF = "
            + arbFF);
  }

  public void setClimberHeight(double height, double arbFF) {
    this.setClimberHeight(height, height, arbFF);
  }

  public void setClimberHeightPct(double lPctHeight, double rPctHeight, double arbFF) {
    setClimberHeight(lPctHeight * _lClimberMaxHeight, rPctHeight * _rClimberMaxHeight, arbFF);
  }

  public void setClimberHeightPct(double pctHeight, double arbFF) {
    setClimberHeight(pctHeight * _lClimberMaxHeight, arbFF);
  }

  public void setClimberHeightInc(double lPctInc, double rPctInc, double arbFF) {
    setClimberHeight(_targetLClimberHeight + lPctInc, _targetRClimberHeight + rPctInc, arbFF);
  }

  public void setClimberHeightInc(double pctInc, double arbFF) {
    setClimberHeight(_targetLClimberHeight + pctInc, arbFF);
  }

  public double getClimberHeight() {
    return smartClimberController.getDistance();
  }

  public double getClimberHeightPct() {
    return getClimberHeight() / _lClimberMaxHeight;
  }

  public boolean isClimbing() {
    return _isClimbing;
  }

  public boolean isResettingClimber() {
    return _isResetLClimber || _isResetRClimber;
  }

  public void setClimberSpeed(double speedL, double speedR, double arbFF) {
    if (!isResettingClimber()) {
      smartClimberController.setSeparateOutput(speedL, speedR, arbFF);
      _isClimbing = false;
      Shuffleboard.addEventMarker("setClimberSpeed: ", EventImportance.kHigh);
    }
  }

  public void setClimberSpeed(double speedL, double speedR) {
    this.setClimberSpeed(speedL, speedR, 0.0);
  }

  public void stop() {
    smartClimberController.setSeparateOutput(0.0, 0.0);
    smartPivotLinkController.setSeparateOutput(0.0, 0.0);
    _targetLClimberHeight = _targetRClimberHeight = this.getClimberHeight();
    _targetLPivotAngle = _targetRPivotAngle = this.getPivotLinkAngle();
  }

  public void reset(double speedC, double speedP) {
    _climbingSeq = 0;
    this.resetClimber(speedC);
    this.resetPivotLink(speedP);
  }

  public void reset() {
    this.reset(Constants.kResetClimberSpeed, Constants.kResetPivotSpeed);
  }

  public void resetClimber(double speed) {
    _isClimbing = false;
    _targetLClimberHeight = _targetRClimberHeight = 0.0;
    _isResetLClimber = _isResetRClimber = true;
    _leftClimberController.configClearPositionOnLimitR(true, 0);
    _rightClimberController.configClearPositionOnLimitR(true, 0);
    smartClimberController.setOutput(isLClimberRevLimitSwitchClosed() ? 0.0 : speed, 0.0);
    smartClimberController.setAuxOutput(isRClimberRevLimitSwitchClosed() ? 0.0 : speed, 0.0);
    Shuffleboard.addEventMarker("resetClimber: ", EventImportance.kHigh);
    System.out.println("resetClimber - start");
  }

  public void resetClimber() {
    this.resetClimber(Constants.kResetClimberSpeed);
  }

  public void zeroClimberPosition() {
    smartClimberController.resetPosition();
    smartPivotLinkController.resetPosition();
    Shuffleboard.addEventMarker("zeroClimberPosition: ", EventImportance.kHigh);
    System.out.println("zeroClimberPosition - start");
  }

  public void setPivotLinkAngle(double lAngleDegrees, double rAngleDegrees) {
    // because max angle is the zero point, clamp angle degrees to min&max, then subtract max to get
    // zeropoint
    _targetLPivotAngle = lAngleDegrees - _maxPivotLinkAngle;
    // MathUtil.clamp(lAngleDegrees, _minPivotLinkAngle, _maxPivotLinkAngle) - _maxPivotLinkAngle;
    _targetRPivotAngle = rAngleDegrees - _maxPivotLinkAngle;
    // MathUtil.clamp(rAngleDegrees, _minPivotLinkAngle, _maxPivotLinkAngle) - _maxPivotLinkAngle;

    smartPivotLinkController.setSeparateTarget(
        _targetLPivotAngle, _targetRPivotAngle, _pivotFeedFwd);
    _isPivoting = true;
    Shuffleboard.addEventMarker("setPivotLinkAngle: ", EventImportance.kHigh);
    System.out.println("setLPivotLinkAngle: " + lAngleDegrees + " tgtAngle: " + _targetLPivotAngle);
    System.out.println("setRPivotLinkAngle: " + rAngleDegrees + " tgtAngle: " + _targetRPivotAngle);
  }

  public void setPivotLinkAngle(double angleDegrees) {
    this.setPivotLinkAngle(angleDegrees, angleDegrees);
  }

  public void setPivotLinkAnglePct(double lPctAngle, double rPctAngle) {
    setPivotLinkAngle(
        (lPctAngle * _pivotLinkAngleRange) + _minPivotLinkAngle,
        (rPctAngle * _pivotLinkAngleRange) + _minPivotLinkAngle);
  }

  public void setPivotLinkAnglePct(double pctAngle) {
    // add angle pct (from 0 to angle swing) to minAngle to get angle
    setPivotLinkAngle((pctAngle * _pivotLinkAngleRange) + _minPivotLinkAngle);
  }

  public void setPivotLinkAngleInc(double lPctInc, double rPctInc) {
    // add angle pct (from 0 to angle swing) to minAngle to get angle
    setPivotLinkAngle(
        _targetLPivotAngle + _maxPivotLinkAngle + lPctInc,
        _targetRPivotAngle + _maxPivotLinkAngle + rPctInc);
  }

  public void setPivotLinkAngleInc(double pctInc) {
    setPivotLinkAngle(_targetLPivotAngle + _maxPivotLinkAngle + pctInc);
  }

  public double getPivotLinkAngle() {
    return smartPivotLinkController.getDistance() + _maxPivotLinkAngle;
  }

  public double getPivotLinkAnglePct() {
    return (getPivotLinkAngle() - _minPivotLinkAngle) / _pivotLinkAngleRange;
  }

  public boolean isPivoting() {
    return _isPivoting;
  }

  public boolean isResettingPivot() {
    return _isResetLPivoting || _isResetRPivoting;
  }

  public void setPivotLinkSpeed(double speedL, double speedR, double arbFF) {
    if (!isResettingPivot()) {
      smartPivotLinkController.setSeparateOutput(speedL, speedR, arbFF);
      _isPivoting = false;
      Shuffleboard.addEventMarker("setPivotLinkSpeed: ", EventImportance.kHigh);
    }
  }

  public void setPivotLinkSpeed(double speedL, double speedR) {
    setPivotLinkSpeed(speedL, speedR, 0.0);
    Shuffleboard.addEventMarker("setPivotLinkSpeed: ", EventImportance.kHigh);
  }

  public void resetPivotLink(double speed) {
    _isPivoting = false;
    _isResetLPivoting = _isResetRPivoting = true;
    _targetLPivotAngle = _targetRPivotAngle = _pivotLinkAngleRange; // reset point is max range
    _leftPivotLinkController.configClearPositionOnLimitF(true, 0);
    _rightPivotLinkController.configClearPositionOnLimitF(true, 0);
    smartPivotLinkController.setOutput(isLPivotFwdLimitSwitchClosed() ? 0.0 : speed, 0.0);
    smartPivotLinkController.setAuxOutput(isRPivotFwdLimitSwitchClosed() ? 0.0 : speed, 0.0);
    Shuffleboard.addEventMarker("resetPivotLink: ", EventImportance.kHigh);
    System.out.println("resetPivotLink - start");
  }

  public void resetPivotLink() {
    this.resetPivotLink(Constants.kResetPivotSpeed);
  }

  public boolean isResetting() {
    return isResettingClimber() || isResettingPivot();
  }

  public boolean isLClimberRevLimitSwitchClosed() {
    return smartClimberController.isRevLimitSwitchClosed() || _isClimberLimitSwitchTest;
  }

  public boolean isLClimberFwdLimitSwitchClosed() {
    return smartClimberController.isFwdLimitSwitchClosed() || _isClimberLimitSwitchTest;
  }

  public boolean isRClimberRevLimitSwitchClosed() {
    return smartClimberController.isAuxRevLimitSwitchClosed() || _isClimberLimitSwitchTest;
  }

  public boolean isRClimberFwdLimitSwitchClosed() {
    return smartClimberController.isAuxFwdLimitSwitchClosed() || _isClimberLimitSwitchTest;
  }

  public boolean isLPivotRevLimitSwitchClosed() {
    return smartPivotLinkController.isRevLimitSwitchClosed() || _isPivotLimitSwitchTest;
  }

  public boolean isLPivotFwdLimitSwitchClosed() {
    return smartPivotLinkController.isFwdLimitSwitchClosed() || _isPivotLimitSwitchTest;
  }

  public boolean isRPivotRevLimitSwitchClosed() {
    return smartPivotLinkController.isAuxRevLimitSwitchClosed() || _isPivotLimitSwitchTest;
  }

  public boolean isRPivotFwdLimitSwitchClosed() {
    return smartPivotLinkController.isAuxFwdLimitSwitchClosed() || _isPivotLimitSwitchTest;
  }

  public void tripClimberLimitSwitches_test(boolean trip) {
    _isClimberLimitSwitchTest = trip;
  }

  public void tripPivotLimitSwitches_test(boolean trip) {
    _isPivotLimitSwitchTest = trip;
  }

  public int nextClimbingSequence() {
    if (_climbingSeq >= 0) {
      _climbingSeq = _climbingSeq + 1;
    }
    return _climbingSeq;
  }

  public int prevClimbingSequence() {
    if (_climbingSeq > 0) {
      _climbingSeq = _climbingSeq - 1;
    }
    return _climbingSeq;
  }
}
