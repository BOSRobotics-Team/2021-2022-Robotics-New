package frc.robot.unused;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import frc.robot.util.*;
import frc.robot.util.sim.PhysicsSim;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX _intakeController = new WPI_TalonSRX(Constants.kID_Intake);
  private final WPI_TalonSRX _liftController = new WPI_TalonSRX(Constants.kID_IntakeLift);

  private final SmartMotor smartIntakeController = new SmartMotor(_intakeController);
  private final SmartMotor smartLiftController = new SmartMotor(_liftController);

  private boolean _isResetLift = false;
  private boolean _isLifting = false;
  private boolean _isFwdLimitSwitchTest = false;
  private boolean _isRevLimitSwitchTest = false;
  private double _targetPos = 0;
  private double kResetLiftSpeed = -0.2;

  private boolean _isIntake = false;
  private double _targetVel = 0.0;

  // private final int kSlotLift = 0;
  // private final int kTimeoutMs = SmartMotor.kTimeoutMs;
  // private final int kPIDLoopIdx = SmartMotor.PID_PRIMARY;

  // private final Convertor _intakeConvertor = new Convertor(4096);
  // private final Convertor _liftConvertor = new Convertor(4096);
  private final GearRatios kGearRatio_Lift;
  private final GearRatios kGearRatio_Intake;

  public static final Gains kDefaultGains_Velocity =
      new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50);
  public static final Gains kDefaultGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.80);

  public Intake() {
    Preferences.initDouble("LiftGearRatio", 12.0);
    Preferences.initDouble("LiftWheelRadius", 1.0);
    Preferences.initDouble("LiftPulleyRatio", 1.0);
    Preferences.initDouble("IntakeGearRatio", 12.0);
    Preferences.initDouble("IntakeWheelRadius", 1.0);
    Preferences.initDouble("IntakePulleyRatio", 1.0);

    kGearRatio_Lift =
        new GearRatios(
            Preferences.getDouble("LiftGearRatio", 12.0),
            Preferences.getDouble("LiftWheelRadius", 1.0),
            Preferences.getDouble("LiftPulleyRatio", 1.0));
    kGearRatio_Intake =
        new GearRatios(
            Preferences.getDouble("IntakeGearRatio", 12.0),
            Preferences.getDouble("IntakeWheelRadius", 1.0),
            Preferences.getDouble("IntakePulleyRatio", 1.0));
    System.out.println("_liftController - ratios = " + kGearRatio_Lift);
    System.out.println("_intakeController - ratios = " + kGearRatio_Intake);

    smartLiftController.initController();
    smartLiftController.configureRatios(kGearRatio_Lift);
    smartLiftController.enableBrakes(true);
    smartLiftController.setDistanceConfigs(kDefaultGains_Distanc);
    smartLiftController.resetSensorPosition();

    // this.configController(_liftController, kDefaultGains_Distanc);
    // _liftController.setNeutralMode(NeutralMode.Brake);
    // _liftConvertor.setRatios(kGearRatio_Lift);

    smartIntakeController.initController();
    smartIntakeController.configureRatios(kGearRatio_Intake);
    smartIntakeController.enableBrakes(true);
    smartIntakeController.setDistanceConfigs(kDefaultGains_Velocity);
    smartIntakeController.resetSensorPosition();

    // this.configController(_intakeController, kDefaultGains_Velocity);
    // _intakeController.setNeutralMode(NeutralMode.Coast);
    // _intakeConvertor.setRatios(kGearRatio_Intake);

    if (RobotBase.isSimulation()) this.simulationInit();
  }

  // public void configController(WPI_TalonSRX _controller, Gains gain) {
  //     _controller.configFactoryDefault();
  //     _controller.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
  // LimitSwitchNormal.NormallyOpen, 0);
  //     _controller.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
  // LimitSwitchNormal.NormallyOpen, 0);
  //     _controller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
  // kPIDLoopIdx, kTimeoutMs);
  //     _controller.configNeutralDeadband(SmartMotor.kNeutralDeadband, kTimeoutMs);
  //     _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
  //     _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);

  //     /* Set the peak and nominal outputs */
  // 	_controller.configNominalOutputForward(0, kTimeoutMs);
  // 	_controller.configNominalOutputReverse(0, kTimeoutMs);
  // 	_controller.configPeakOutputForward(1, kTimeoutMs);
  //     _controller.configPeakOutputReverse(-1, kTimeoutMs);

  // 	/* Set Motion Magic gains in slot0 - see documentation */
  // 	_controller.selectProfileSlot(kSlotLift, kPIDLoopIdx);
  //     _controller.config_kF(kSlotLift, gain.kF, kTimeoutMs);
  // 	_controller.config_kP(kSlotLift, gain.kP, kTimeoutMs);
  // 	_controller.config_kI(kSlotLift, gain.kI, kTimeoutMs);
  // 	_controller.config_kD(kSlotLift, gain.kD, kTimeoutMs);

  // 	/* Set acceleration and vcruise velocity - see documentation */
  //     _controller.configMotionCruiseVelocity(3000, kTimeoutMs);
  //     _controller.configMotionAcceleration(3000, kTimeoutMs);

  //     _controller.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
  // }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if (_isResetLift) {
      if (isLiftRevLimitSwitchClosed()) {
        _isResetLift = false;
        _isLifting = false;
        smartLiftController.set(0.0);
        smartLiftController.resetSensorPosition();
      }
    }
    if (_isLifting) {
      System.out.println("isLifting - current pos = " + smartLiftController.getPosition());

      if (Math.abs(smartLiftController.getPosition() - _targetPos) < 0.01) {
        _isLifting = false;
        System.out.println("isLifting - done");
      } else if (isLiftFwdLimitSwitchClosed()) {
        _isLifting = false;
        System.out.println("isLifting - limit exceeded");
      }
    }
    if (_isIntake) {
      System.out.println("isIntake - current vel = " + smartIntakeController.getVelocity());
      smartIntakeController.setVelocity(_targetVel);
    }
    smartIntakeController.update();
    smartLiftController.update();
  }

  public void simulationInit() {
    if (RobotBase.isSimulation())
      PhysicsSim.getInstance().addTalonSRX((WPI_TalonSRX) _intakeController, 0.2, 6800);
    if (RobotBase.isSimulation())
      PhysicsSim.getInstance().addTalonSRX((WPI_TalonSRX) _liftController, 0.2, 6800);
    if (RobotBase.isSimulation()) {
      PhysicsSim.getInstance().run();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void logPeriodic() {
    // _intakeController.logPeriodic();
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void runIntake(double speed) {
    _targetVel = speed;
    _isIntake = true;

    System.out.println("isIntake - target (meters/sec) = " + speed);
  }

  public void stopIntake() {
    _isIntake = false;
    smartIntakeController.set(0.0);
    System.out.println("isIntake - stopped ");
  }

  public void runLift(double height) {
    _targetPos = height;
    smartLiftController.setTarget(_targetPos);
    _isLifting = true;

    // System.out.println("isLifting - target (meters) = " + height);
  }

  public boolean isLifting() {
    return _isLifting;
  }

  public void setLift(double speed) {
    if (!_isLifting && !_isResetLift) {
      smartLiftController.set(speed);
    }
  }

  public void resetLift() {
    _isResetLift = true;
    smartLiftController.set(kResetLiftSpeed);
  }

  public boolean isResetting() {
    return _isResetLift;
  }

  public boolean isLiftFwdLimitSwitchClosed() {
    return smartLiftController.isFwdLimitSwitchClosed() || _isFwdLimitSwitchTest;
  }

  public boolean isLiftRevLimitSwitchClosed() {
    return smartLiftController.isRevLimitSwitchClosed() || _isRevLimitSwitchTest;
  }

  public void tripFwdLimitSwitches_test(boolean trip) {
    _isFwdLimitSwitchTest = trip;
  }

  public void tripRevLimitSwitches_test(boolean trip) {
    System.out.println("tripRevLimitSwitches_test -" + trip);
    _isRevLimitSwitchTest = trip;
  }
}
