// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
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

  private double _targetLClimberHeight = 0;
  private double _targetRClimberHeight = 0;
  private double _lClimberMaxHeight = Constants.kLClimberMaxHeight;
  private double _rClimberMaxHeight = Constants.kRClimberMaxHeight;

  private boolean _isPivoting = false;
  private boolean _isResetLPivoting = false;
  private boolean _isResetRPivoting = false;
  private double _targetLPivotAngle = 0;
  private double _targetRPivotAngle = 0;
  private double _lPivotLinkMinAngle = Constants.kLPivotLinkMinAngle;
  private double _lPivotLinkMaxAngle = Constants.kLPivotLinkMaxAngle;
  private double _rPivotLinkMinAngle = Constants.kRPivotLinkMinAngle;
  private double _rPivotLinkMaxAngle = Constants.kRPivotLinkMaxAngle;
  private double _lPivotLinkAngleRange = (_lPivotLinkMaxAngle - _lPivotLinkMinAngle);
  private double _rPivotLinkAngleRange = (_rPivotLinkMaxAngle - _rPivotLinkMinAngle);
  private double _pivotFeedFwd = Constants.kPivotLinkFeedFwd;

  private ClimberSim _climberSim = null;

  public Climber() {
    // Preferences.initDouble("LClimberMaxHeight", Constants.kLClimberMaxHeight);
    // Preferences.initDouble("RClimberMaxHeight", Constants.kLClimberMaxHeight);
    // _lClimberMaxHeight = Preferences.getDouble("LClimberMaxHeight",
    // Constants.kLClimberMaxHeight);
    // _rClimberMaxHeight = Preferences.getDouble("RClimberMaxHeight",
    // Constants.kLClimberMaxHeight);

    _leftClimberController.setInverted(InvertType.None);
    _rightClimberController.setInverted(InvertType.InvertMotorOutput);
    smartClimberController.initController();
    smartClimberController.configureRatios(Constants.kClimberGearRatio);
    smartClimberController.enableBrakes(true);
    smartClimberController.setSeparateDistanceConfigs(Constants.kClimberGains_Distance);
    _leftClimberController.configForwardSoftLimitThreshold(
        smartClimberController.convertor.distanceMetersToNativeUnits(_lClimberMaxHeight));
    _rightClimberController.configForwardSoftLimitThreshold(
        smartClimberController.convertor.distanceMetersToNativeUnits(_rClimberMaxHeight));

    _leftPivotLinkController.setInverted(InvertType.None);
    _rightPivotLinkController.setInverted(InvertType.InvertMotorOutput);
    smartPivotLinkController.initController();
    smartPivotLinkController.configureRatios(Constants.kPivotLinkGearRatio);
    smartPivotLinkController.enableBrakes(true);
    smartPivotLinkController.setSeparateDistanceConfigs(Constants.kPivotLinkGains_Distance);
    _leftPivotLinkController.configReverseSoftLimitThreshold(
        smartPivotLinkController.convertor.distanceMetersToNativeUnits(
            -(_lPivotLinkAngleRange + 5.0)));
    _rightPivotLinkController.configReverseSoftLimitThreshold(
        smartPivotLinkController.convertor.distanceMetersToNativeUnits(
            -(_rPivotLinkAngleRange + 5.0)));

    if (RobotBase.isSimulation()) this.simulationInit();

    this.zeroClimberPosition();
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop

    if (_isResetLClimber && isLClimberRevLimitSwitchClosed()) {
      _isResetLClimber = false;
      smartClimberController.setOutput(0.0, 0.0);
      _leftClimberController.configClearPositionOnLimitR(false, 0);
      _leftClimberController.configForwardSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetLClimber - done: ", EventImportance.kHigh);
      System.out.println("isResetLClimber - done");
    }
    if (_isResetRClimber && isRClimberRevLimitSwitchClosed()) {
      _isResetRClimber = false;
      smartClimberController.setAuxOutput(0.0, 0.0);
      _rightClimberController.configClearPositionOnLimitR(false, 0);
      _rightClimberController.configForwardSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetRClimber - done: ", EventImportance.kHigh);
      System.out.println("isResetRClimber - done");
    }
    if (_isResetLPivoting && isLPivotFwdLimitSwitchClosed()) {
      _isResetLPivoting = false;
      smartPivotLinkController.setOutput(0.0, 0.0);
      _leftPivotLinkController.configClearPositionOnLimitF(false, 0);
      _leftPivotLinkController.configReverseSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetLPivot - done: ", EventImportance.kHigh);
      System.out.println("isResetLPivot - done");
    }
    if (_isResetRPivoting && isRPivotFwdLimitSwitchClosed()) {
      _isResetRPivoting = false;
      smartPivotLinkController.setAuxOutput(0.0, 0.0);
      _rightPivotLinkController.configClearPositionOnLimitF(false, 0);
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
            _rClimberMaxHeight,
            _leftPivotLinkController,
            _rightPivotLinkController,
            smartPivotLinkController.convertor,
            Constants.kPivotLinkMass,
            Constants.kPivotLinkLength,
            _lPivotLinkMinAngle,
            _lPivotLinkMaxAngle,
            _rPivotLinkMinAngle,
            _rPivotLinkMaxAngle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    _climberSim.run();
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
    this.setClimberHeight(lPctHeight * _lClimberMaxHeight, rPctHeight * _rClimberMaxHeight, arbFF);
  }

  public void setClimberHeightPct(double pctHeight, double arbFF) {
    this.setClimberHeightPct(pctHeight, pctHeight, arbFF);
  }

  public void setClimberHeightInc(double lPctInc, double rPctInc, double arbFF) {
    this.setClimberHeight(_targetLClimberHeight + lPctInc, _targetRClimberHeight + rPctInc, arbFF);
  }

  public void setClimberHeightInc(double pctInc, double arbFF) {
    this.setClimberHeightInc(pctInc, pctInc, arbFF);
  }

  public double getLClimberHeight() {
    return smartClimberController.getDistance();
  }

  public double getRClimberHeight() {
    return smartClimberController.getAuxDistance();
  }

  public double getLClimberHeightPct() {
    return getLClimberHeight() / _lClimberMaxHeight;
  }

  public double getRClimberHeightPct() {
    return getRClimberHeight() / _rClimberMaxHeight;
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
    _targetLClimberHeight = _targetRClimberHeight = this.getLClimberHeight();
    _targetLPivotAngle = _targetRPivotAngle = this.getLPivotLinkAngle();
    _isResetLClimber = _isResetRClimber = false;
    _isResetLPivoting = _isResetRPivoting = false;
    _isClimbing = _isPivoting = false;
    smartClimberController.setSeparateOutput(0.0, 0.0);
    smartPivotLinkController.setSeparateOutput(0.0, 0.0);
  }

  public void reset(double speedC, double speedP) {
    _climbingSeq = -1;
    this.resetClimber(speedC);
    this.resetPivotLink(speedP);
  }

  public void reset() {
    this.reset(Constants.kResetClimberSpeed, Constants.kResetPivotSpeed);
  }

  public void startClimbingSequence() {
    _climbingSeq = 0;
  }

  public void resetClimber(double speed) {
    _isClimbing = false;
    _targetLClimberHeight = _targetRClimberHeight = 0.0;
    _isResetLClimber = _isResetRClimber = true;

    smartClimberController.setOutput(isLClimberRevLimitSwitchClosed() ? 0.0 : speed, 0.0);
    smartClimberController.setAuxOutput(isRClimberRevLimitSwitchClosed() ? 0.0 : speed, 0.0);
    smartClimberController.configClearPositionOnLimitR(true);
    smartClimberController.configForwardSoftLimitEnable(false);

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
    _targetLPivotAngle = lAngleDegrees - _lPivotLinkMaxAngle;
    // MathUtil.clamp(lAngleDegrees, _minPivotLinkAngle, _maxPivotLinkAngle) - _maxPivotLinkAngle;
    _targetRPivotAngle = rAngleDegrees - _rPivotLinkMaxAngle;
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
    // add angle pct (from 0 to angle swing) to minAngle to get angle
    this.setPivotLinkAngle(
        (lPctAngle * _lPivotLinkAngleRange) + _lPivotLinkMinAngle,
        (rPctAngle * _rPivotLinkAngleRange) + _rPivotLinkMinAngle);
  }

  public void setPivotLinkAnglePct(double pctAngle) {
    this.setPivotLinkAnglePct(pctAngle, pctAngle);
  }

  public void setPivotLinkAngleInc(double lPctInc, double rPctInc) {
    // add angle pct (from 0 to angle swing) to minAngle to get angle
    this.setPivotLinkAngle(
        _targetLPivotAngle + _lPivotLinkMaxAngle + lPctInc,
        _targetRPivotAngle + _rPivotLinkMaxAngle + rPctInc);
  }

  public void setPivotLinkAngleInc(double pctInc) {
    this.setPivotLinkAngleInc(pctInc, pctInc);
  }

  public double getLPivotLinkAngle() {
    return smartPivotLinkController.getDistance() + _lPivotLinkMaxAngle;
  }

  public double getRPivotLinkAngle() {
    return smartPivotLinkController.getAuxDistance() + _rPivotLinkMaxAngle;
  }

  public double getLPivotLinkAnglePct() {
    return (getLPivotLinkAngle() - _lPivotLinkMinAngle) / _lPivotLinkAngleRange;
  }

  public double getRPivotLinkAnglePct() {
    return (getRPivotLinkAngle() - _rPivotLinkMinAngle) / _rPivotLinkAngleRange;
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
    _targetLPivotAngle = _lPivotLinkAngleRange;
    _targetRPivotAngle = _rPivotLinkAngleRange; // reset point is max range
    this.lockPivotLinksForDriving(false);
    smartPivotLinkController.setOutput(isLPivotFwdLimitSwitchClosed() ? 0.0 : speed, 0.0);
    smartPivotLinkController.setAuxOutput(isRPivotFwdLimitSwitchClosed() ? 0.0 : speed, 0.0);
    smartPivotLinkController.configClearPositionOnLimitF(true);
    smartPivotLinkController.configReverseSoftLimitEnable(false);

    Shuffleboard.addEventMarker("resetPivotLink: ", EventImportance.kHigh);
    System.out.println("resetPivotLink - start");
  }

  public void resetPivotLink() {
    this.resetPivotLink(Constants.kResetPivotSpeed);
  }

  public void lockPivotLinksForDriving(boolean lock) {
    // smartPivotLinkController.overrideLimitSwitchesEnable(lock);
    if (lock) {
      this.setPivotLinkAnglePct(1.0);
    } else {
      smartPivotLinkController.set(0.0);
    }
    Shuffleboard.addEventMarker("lockPivotLinksForDriving: ", EventImportance.kHigh);
    System.out.println("lockPivotLinksForDriving - " + lock);
  }

  public boolean isResetting() {
    return isResettingClimber() || isResettingPivot();
  }

  public boolean isLClimberRevLimitSwitchClosed() {
    return smartClimberController.isRevLimitSwitchClosed();
  }

  public boolean isLClimberFwdLimitSwitchClosed() {
    return smartClimberController.isFwdLimitSwitchClosed();
  }

  public boolean isRClimberRevLimitSwitchClosed() {
    return smartClimberController.isAuxRevLimitSwitchClosed();
  }

  public boolean isRClimberFwdLimitSwitchClosed() {
    return smartClimberController.isAuxFwdLimitSwitchClosed();
  }

  public boolean isLPivotRevLimitSwitchClosed() {
    return smartPivotLinkController.isRevLimitSwitchClosed();
  }

  public boolean isLPivotFwdLimitSwitchClosed() {
    return smartPivotLinkController.isFwdLimitSwitchClosed();
  }

  public boolean isRPivotRevLimitSwitchClosed() {
    return smartPivotLinkController.isAuxRevLimitSwitchClosed();
  }

  public boolean isRPivotFwdLimitSwitchClosed() {
    return smartPivotLinkController.isAuxFwdLimitSwitchClosed();
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
