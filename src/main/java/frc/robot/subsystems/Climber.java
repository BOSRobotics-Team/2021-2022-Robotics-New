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
import frc.robot.util.*;
import frc.robot.util.sim.ClimberSim;

public class Climber extends SubsystemBase {
  private final WPI_TalonFX _leftClimber = new WPI_TalonFX(Constants.kID_LClimber);
  private final WPI_TalonFX _rightClimber = new WPI_TalonFX(Constants.kID_RClimber);
  private final WPI_TalonFX _leftPivotLink = new WPI_TalonFX(Constants.kID_LPivot);
  private final WPI_TalonFX _rightPivotLink = new WPI_TalonFX(Constants.kID_RPivot);

  private final SmartMotor _leftClimberController = new SmartMotor(_leftClimber, "LClimber");
  private final SmartMotor _rightClimberController = new SmartMotor(_rightClimber, "RClimber");
  private final SmartMotor _leftPivotLinkController = new SmartMotor(_leftPivotLink, "LPivot");
  private final SmartMotor _rightPivotLinkController = new SmartMotor(_rightPivotLink, "RPivot");

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
    _leftClimberController.initController();
    _leftClimberController.configureRatios(Constants.kClimberGearRatio);
    _leftClimberController.enableBrakes(true);
    _leftClimberController.setDistanceConfigs(Constants.kClimberGains_Distance);
    _leftClimberController.configForwardSoftLimitDistance(_lClimberMaxHeight);

    _rightClimberController.setInverted(InvertType.InvertMotorOutput);
    _rightClimberController.initController();
    _rightClimberController.configureRatios(Constants.kClimberGearRatio);
    _rightClimberController.enableBrakes(true);
    _rightClimberController.setDistanceConfigs(Constants.kClimberGains_Distance);
    _rightClimberController.configForwardSoftLimitDistance(_rClimberMaxHeight);

    _leftPivotLinkController.setInverted(InvertType.None);
    _leftPivotLinkController.initController();
    _leftPivotLinkController.configureRatios(Constants.kPivotLinkGearRatio);
    _leftPivotLinkController.enableBrakes(true);
    _leftPivotLinkController.setDistanceConfigs(Constants.kPivotLinkGains_Distance);
    _leftPivotLinkController.configReverseSoftLimitDistance(-(_lPivotLinkAngleRange + 5.0));

    _rightPivotLinkController.setInverted(InvertType.InvertMotorOutput);
    _rightPivotLinkController.initController();
    _rightPivotLinkController.configureRatios(Constants.kPivotLinkGearRatio);
    _rightPivotLinkController.enableBrakes(true);
    _rightPivotLinkController.setDistanceConfigs(Constants.kPivotLinkGains_Distance);
    _rightPivotLinkController.configReverseSoftLimitThreshold(-(_rPivotLinkAngleRange + 5.0));

    if (RobotBase.isSimulation()) this.simulationInit();

    this.zeroClimberPosition();
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop

    if (_isResetLClimber && isLClimberRevLimitSwitchClosed()) {
      _isResetLClimber = false;
      _leftClimberController.set(0.0);
      _leftClimberController.configClearPositionOnLimitR(false);
      _leftClimberController.configForwardSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetLClimber - done: ", EventImportance.kHigh);
      System.out.println("isResetLClimber - done");
    }
    if (_isResetRClimber && isRClimberRevLimitSwitchClosed()) {
      _isResetRClimber = false;
      _rightClimberController.set(0.0);
      _rightClimberController.configClearPositionOnLimitR(false);
      _rightClimberController.configForwardSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetRClimber - done: ", EventImportance.kHigh);
      System.out.println("isResetRClimber - done");
    }
    if (_isResetLPivoting && isLPivotFwdLimitSwitchClosed()) {
      _isResetLPivoting = false;
      _leftPivotLinkController.set(0.0);
      _leftPivotLinkController.configClearPositionOnLimitF(false);
      _leftPivotLinkController.configReverseSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetLPivot - done: ", EventImportance.kHigh);
      System.out.println("isResetLPivot - done");
    }
    if (_isResetRPivoting && isRPivotFwdLimitSwitchClosed()) {
      _isResetRPivoting = false;
      _leftPivotLinkController.set(0.0);
      _leftPivotLinkController.configClearPositionOnLimitF(false);
      _leftPivotLinkController.configReverseSoftLimitEnable(true);
      Shuffleboard.addEventMarker("isResetRPivot - done: ", EventImportance.kHigh);
      System.out.println("isResetRPivot - done");
    }
    if (_isClimbing) {
      if (_leftClimberController.isTargetFinished() && _rightClimberController.isTargetFinished()) {
        _isClimbing = false;
        Shuffleboard.addEventMarker("isClimbing - done: ", EventImportance.kHigh);
        System.out.println("isClimbing - done");
      }
    }
    if (_isPivoting) {
      if (_leftPivotLinkController.isTargetFinished()
          && _rightPivotLinkController.isTargetFinished()) {
        _isPivoting = false;
        Shuffleboard.addEventMarker("isPivoting - done: ", EventImportance.kHigh);
        System.out.println("isPivoting - done");
      }
    }
    _leftClimberController.update();
    _rightClimberController.update();
    _leftPivotLinkController.update();
    _rightPivotLinkController.update();
  }

  public void simulationInit() {
    _climberSim =
        new ClimberSim(
            _leftClimberController,
            _rightClimberController,
            0.1, // Constants.kDriveCarriageMass,
            0.0,
            0.0,
            _lClimberMaxHeight,
            _rClimberMaxHeight,
            _leftPivotLinkController,
            _rightPivotLinkController,
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
    _leftClimberController.logPeriodic();
    _rightClimberController.logPeriodic();
    _leftPivotLinkController.logPeriodic();
    _rightPivotLinkController.logPeriodic();
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

    _leftClimberController.setTargetWithFF(_targetLClimberHeight, arbFF);
    _rightClimberController.setTargetWithFF(_targetRClimberHeight, arbFF);
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
    return _leftClimberController.getDistance();
  }

  public double getRClimberHeight() {
    return _rightClimberController.getDistance();
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
      _leftClimberController.set(speedL, arbFF);
      _rightClimberController.set(speedR, arbFF);
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
    _leftClimberController.set(0.0);
    _rightClimberController.set(0.0);
    _leftPivotLinkController.set(0.0);
    _rightPivotLinkController.set(0.0);
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

    _leftClimberController.set(isLClimberRevLimitSwitchClosed() ? 0.0 : speed, 0.0);
    _leftClimberController.configClearPositionOnLimitR(true);
    _leftClimberController.configForwardSoftLimitEnable(false);

    _rightClimberController.set(isRClimberRevLimitSwitchClosed() ? 0.0 : speed, 0.0);
    _rightClimberController.configClearPositionOnLimitR(true);
    _rightClimberController.configForwardSoftLimitEnable(false);

    Shuffleboard.addEventMarker("resetClimber: ", EventImportance.kHigh);
    System.out.println("resetClimber - start");
  }

  public void resetClimber() {
    this.resetClimber(Constants.kResetClimberSpeed);
  }

  public void zeroClimberPosition() {
    _leftClimberController.resetSensorPosition();
    _rightClimberController.resetSensorPosition();
    _leftPivotLinkController.resetSensorPosition();
    _rightPivotLinkController.resetSensorPosition();
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

    _leftPivotLinkController.setTargetWithFF(_targetLPivotAngle, _pivotFeedFwd);
    _rightPivotLinkController.setTargetWithFF(_targetRPivotAngle, _pivotFeedFwd);
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
    return _leftPivotLinkController.getDistance() + _lPivotLinkMaxAngle;
  }

  public double getRPivotLinkAngle() {
    return _rightPivotLinkController.getDistance() + _rPivotLinkMaxAngle;
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
      _leftPivotLinkController.set(speedL, arbFF);
      _rightPivotLinkController.set(speedR, arbFF);
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
    _leftPivotLinkController.set(isLPivotFwdLimitSwitchClosed() ? 0.0 : speed, 0.0);
    _leftPivotLinkController.configClearPositionOnLimitF(true);
    _leftPivotLinkController.configReverseSoftLimitEnable(false);

    _rightPivotLinkController.set(isRPivotFwdLimitSwitchClosed() ? 0.0 : speed, 0.0);
    _rightPivotLinkController.configClearPositionOnLimitF(true);
    _rightPivotLinkController.configReverseSoftLimitEnable(false);

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
      _leftPivotLinkController.set(0.0);
      _rightPivotLinkController.set(0.0);
    }
    Shuffleboard.addEventMarker("lockPivotLinksForDriving: ", EventImportance.kHigh);
    System.out.println("lockPivotLinksForDriving - " + lock);
  }

  public boolean isResetting() {
    return isResettingClimber() || isResettingPivot();
  }

  public boolean isLClimberRevLimitSwitchClosed() {
    return _leftClimberController.isRevLimitSwitchClosed();
  }

  public boolean isLClimberFwdLimitSwitchClosed() {
    return _leftClimberController.isFwdLimitSwitchClosed();
  }

  public boolean isRClimberRevLimitSwitchClosed() {
    return _rightClimberController.isRevLimitSwitchClosed();
  }

  public boolean isRClimberFwdLimitSwitchClosed() {
    return _rightClimberController.isFwdLimitSwitchClosed();
  }

  public boolean isLPivotRevLimitSwitchClosed() {
    return _leftPivotLinkController.isRevLimitSwitchClosed();
  }

  public boolean isLPivotFwdLimitSwitchClosed() {
    return _leftPivotLinkController.isFwdLimitSwitchClosed();
  }

  public boolean isRPivotRevLimitSwitchClosed() {
    return _rightPivotLinkController.isRevLimitSwitchClosed();
  }

  public boolean isRPivotFwdLimitSwitchClosed() {
    return _rightPivotLinkController.isFwdLimitSwitchClosed();
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
