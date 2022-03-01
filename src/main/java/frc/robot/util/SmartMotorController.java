package frc.robot.util;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SmartMotorController {
  /** ---- Flat constants, you should not need to change these ---- */
  public static final int kTalonNone = 0;

  public static final int kTalonSRX = 1;
  public static final int kTalonFX = 2;

  /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
  public static final int REMOTE_0 = 0;
  public static final int REMOTE_1 = 1;
  /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
  public static final int PID_PRIMARY = 0;
  public static final int PID_AUX = 1;
  public static final int PID_DISTANCE = 0;
  public static final int PID_TURN = 1;
  /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
  public static final int SLOT_0 = 0;
  public static final int SLOT_1 = 1;
  public static final int SLOT_2 = 2;
  public static final int SLOT_3 = 3;
  /* ---- Named slots, used to clarify code ---- */
  public static final int kSlot_Distanc = SLOT_0;
  public static final int kSlot_Turning = SLOT_1;
  public static final int kSlot_Velocit = SLOT_2;
  public static final int kSlot_MotProf = SLOT_3;

  public static final int kTimeoutMs = 30;
  public static final double kNeutralDeadband = 0.001;

  public static final int kSensorUnitsPerRotation[] = {4096, 4096, 2048};
  public static final FeedbackDevice kDefaultFeedbackDevice[] = {
    FeedbackDevice.None, FeedbackDevice.CTRE_MagEncoder_Relative, FeedbackDevice.IntegratedSensor
  };

  public static final GearRatios kDefaultGearRatio =
      new GearRatios(Constants.kDriveGearRatio, Constants.kDriveWheelRadiusInches, 1.0);
  public static final Gains kDefaultGains_Distanc = new Gains(0.0, 0.0, 0.0, 0.0, 100, 1.0);

  /**
   * Empirically measure what the difference between encoders per 360' Drive the robot in clockwise
   * rotations and measure the units per rotation. Drive the robot in counter clockwise rotations
   * and measure the units per rotation. Take the average of the two.
   */
  public static final int kEncoderUnitsPerRotation = 51711;
  /**
   * Using the configSelectedFeedbackCoefficient() function, scale units to 3600 per rotation. This
   * is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
   */
  public static final double kTurnTravelUnitsPerRotation = 3600.0;

  public static final double kFeedbackCoefficient =
      kTurnTravelUnitsPerRotation / kEncoderUnitsPerRotation;

  public final String name;
  public final Convertor convertor;

  private final int _controllerType;
  private final BaseTalon _controller;
  private final BaseTalon _auxController;
  private final boolean _hasAuxController;

  private final Faults _faults = new Faults();

  private SimpleMotorFeedforward _feedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);
  private FeedbackDevice _feedbackDevice = FeedbackDevice.None;

  private enum SetPointMode {
    None,
    Distance,
    DistanceFollow,
    DistanceAux,
    Velocity,
    Finished
  };

  private SetPointMode _mode = SetPointMode.None;
  private double _setpoint = 0.0; // The most recently set setpoint.
  private double _nativeSetpoint = 0.0; // The setpoint in native units.
  private double _auxpoint = 0.0; // The most recently set auxpoint.
  private double _nativeAuxpoint = 0.0; // The auxpoint in native units.
  private double _currentTrajPos = 0.0; // The current trajectory position.
  private double _currentAuxTrajPos = 0.0; // The current aux trajectory position.

  public SmartMotorController(final BaseTalon talon, final BaseTalon auxTalon, final String nm) {
    name = nm;

    if (talon.getClass() == WPI_TalonFX.class) {
      _controllerType = kTalonFX;
    } else if (talon.getClass() == WPI_TalonSRX.class) {
      _controllerType = kTalonSRX;
    } else {
      _controllerType = kTalonNone;
    }
    convertor = new Convertor(kSensorUnitsPerRotation[_controllerType]);
    convertor.setRatios(kDefaultGearRatio);

    _controller = talon;
    _auxController = auxTalon;

    _controller.configFactoryDefault();

    _hasAuxController = (_auxController != null);
    if (_hasAuxController) {
      _auxController.configFactoryDefault();
    }

    _feedbackDevice = kDefaultFeedbackDevice[_controllerType];
  }

  public SmartMotorController(final BaseTalon talon, final String name) {
    this(talon, null, name);
  }

  public void initController() {
    _mode = SetPointMode.None;
    initController(_controller);
    if (_hasAuxController) {
      initController(_auxController);
    }
    this.setSeparateDistanceConfigs(SmartMotorController.kDefaultGains_Distanc);
  }

  public void configureRatios(final GearRatios gearRatio) {
    convertor.setRatios(gearRatio);
  }

  public void configureFeedForward(final double gain, final double velGain) {
    _feedForwardCalculator = new SimpleMotorFeedforward(gain, velGain);
  }

  public void configureFeedForward(final double gain) {
    configureFeedForward(gain, 0);
  }

  private void initController(final BaseTalon talon) {
    /* Configure Sensor Source for Primary PID */
    talon.configSelectedFeedbackSensor(_feedbackDevice, PID_PRIMARY, kTimeoutMs);

    /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
    talon.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

    // Set the peak and nominal outputs
    talon.configNominalOutputForward(0, kTimeoutMs);
    talon.configNominalOutputReverse(0, kTimeoutMs);
    talon.configPeakOutputForward(1, kTimeoutMs);
    talon.configPeakOutputReverse(-1, kTimeoutMs);

    talon.configClearPositionOnLimitF(false, kTimeoutMs);
    talon.configClearPositionOnLimitR(false, kTimeoutMs);
    talon.configForwardSoftLimitEnable(false, kTimeoutMs);
    talon.configReverseSoftLimitEnable(false, kTimeoutMs);

    if (_controllerType == kTalonFX) {
      talon.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, kTimeoutMs);
      talon.configReverseLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, kTimeoutMs);
    }
  }

  private void setClosedLoopGains(final BaseTalon talon, final int slot, final Gains gain) {
    talon.config_kP(slot, gain.kP, kTimeoutMs);
    talon.config_kI(slot, gain.kI, kTimeoutMs);
    talon.config_kD(slot, gain.kD, kTimeoutMs);
    talon.config_kF(slot, gain.kF, kTimeoutMs);
    talon.config_IntegralZone(slot, gain.kIzone, kTimeoutMs);
    talon.configClosedLoopPeakOutput(slot, gain.kPeakOutput);
    talon.configAllowableClosedloopError(slot, 0, kTimeoutMs);
    talon.configClosedLoopPeriod(slot, 1);
  }

  private void setDistanceConfigs(final BaseTalon talon, final Gains gains) {
    this.setClosedLoopGains(talon, kSlot_Distanc, gains);

    // Set acceleration and vcruise velocity - see documentation
    talon.configMotionAcceleration(gains.kMotionAccel, kTimeoutMs);
    talon.configMotionCruiseVelocity(gains.kMotionCruiseVel, kTimeoutMs);
    talon.configMotionSCurveStrength(gains.kMotionSCurve, kTimeoutMs);

    talon.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
    talon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, kTimeoutMs);
    talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, kTimeoutMs);

    /* Configure Sensor Source for Primary PID */
    talon.configSelectedFeedbackSensor(_feedbackDevice, PID_PRIMARY, kTimeoutMs);
  }

  public void setSeparateDistanceConfigs(Gains gains) {
    this.setDistanceConfigs(_controller, gains);
    if (_hasAuxController) this.setDistanceConfigs(_auxController, gains);
  }

  public void setDistanceConfigs(final Gains gains) {
    this.setDistanceConfigs(_controller, gains);

    if (_hasAuxController) {
      this.setDistanceConfigs(_auxController, gains);

      _controller.configRemoteFeedbackFilter(
          _auxController.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, REMOTE_0);

      /* Check if we're inverted */
      if (_controller.getInverted()) {
        _controller.configSensorTerm(SensorTerm.Diff0, _feedbackDevice);
        _controller.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0);
        _controller.configSelectedFeedbackSensor(
            FeedbackDevice.SensorDifference, PID_PRIMARY, kTimeoutMs);
      } else {
        _controller.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
        _controller.configSensorTerm(SensorTerm.Sum1, _feedbackDevice);
        _controller.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_PRIMARY, kTimeoutMs);
      }
      _controller.configAuxPIDPolarity(false, kTimeoutMs);
      _controller.configSelectedFeedbackCoefficient(0.5, PID_PRIMARY, kTimeoutMs);
    }
  }

  public void setDistanceAndTurnConfigs(final Gains dgains, final Gains tgains) {
    this.setDistanceConfigs(dgains);

    this.setClosedLoopGains(_controller, kSlot_Turning, tgains);

    _controller.selectProfileSlot(kSlot_Turning, PID_TURN);
    _controller.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10, kTimeoutMs);

    if (_hasAuxController) {
      setClosedLoopGains(_auxController, kSlot_Turning, tgains);
      _auxController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10, kTimeoutMs);

      /* Check if we're inverted */
      if (_controller.getInverted() != _auxController.getInverted()) {
        _controller.configSensorTerm(SensorTerm.Sum0, _feedbackDevice);
        _controller.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor0);
        _controller.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_TURN, kTimeoutMs);
        _controller.configAuxPIDPolarity(true, kTimeoutMs);
      } else {
        _controller.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0);
        _controller.configSensorTerm(SensorTerm.Diff1, _feedbackDevice);
        _controller.configSelectedFeedbackSensor(
            FeedbackDevice.SensorDifference, PID_TURN, kTimeoutMs);
        _controller.configAuxPIDPolarity(true, kTimeoutMs);
      }
      _controller.configSelectedFeedbackCoefficient(kFeedbackCoefficient, PID_TURN, kTimeoutMs);
    }
  }

  public void setVelocityConfigs(final Gains gains) {
    setClosedLoopGains(_controller, kSlot_Velocit, gains);

    _controller.selectProfileSlot(kSlot_Velocit, PID_PRIMARY);
    _controller.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10);
    _controller.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, kTimeoutMs);

    /* Configure Sensor Source for Primary PID */
    _controller.configSelectedFeedbackSensor(_feedbackDevice, PID_PRIMARY, kTimeoutMs);

    if (_hasAuxController) {
      setClosedLoopGains(_auxController, kSlot_Velocit, gains);

      _controller.configRemoteFeedbackFilter(
          _auxController.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, REMOTE_0);
      /* Check if we're inverted */
      if (_controller.getInverted() != _auxController.getInverted()) {
        _controller.configSensorTerm(SensorTerm.Diff0, _feedbackDevice);
        _controller.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0);
        _controller.configSelectedFeedbackSensor(
            FeedbackDevice.SensorDifference, PID_PRIMARY, kTimeoutMs);
      } else {
        _controller.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
        _controller.configSensorTerm(SensorTerm.Sum1, _feedbackDevice);
        _controller.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_PRIMARY, kTimeoutMs);
      }
      _controller.configSelectedFeedbackCoefficient(0.5, PID_PRIMARY, kTimeoutMs);
    }
  }

  /**
   * Get the position of the drive.
   *
   * @return The signed position in units, or null if the drive doesn't have encoders.
   */
  public Double getPosition() {
    return _controller.getSelectedSensorPosition(PID_PRIMARY);
  }

  /**
   * Get the position of the drive in meters.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  public Double getDistance() {
    return convertor.nativeUnitsToDistanceMeters(getPosition());
  }

  /**
   * Get the aux position of the drive.
   *
   * @return The signed position in units, or null if the drive doesn't have encoders.
   */
  public Double getTurnPosition() {
    return _controller.getSelectedSensorPosition(PID_TURN);
  }

  /**
   * Get the aux position of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  public Double getTurnDistance() {
    return convertor.nativeUnitsToDistanceMeters(getAuxPosition());
  }

  /**
   * Get the velocity of the drive.
   *
   * @return The signed velocity in units per second, or null if the drive doesn't have encoders.
   */
  public Double getVelocityUPS() {
    return _controller.getSelectedSensorVelocity(PID_PRIMARY);
  }

  /**
   * Get the velocity of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  public Double getVelocity() {
    return convertor.nativeUnitsToVelocity(getVelocityUPS());
  }

  /**
   * Get the angular velocity of the drive.
   *
   * @return The signed velocity in units per second, or null if the drive doesn't have encoders.
   */
  public Double getTurnVelocityUPS() {
    return _controller.getSelectedSensorVelocity(PID_TURN);
  }

  /**
   * Get the angular velocity of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  public Double getTurnVelocity() {
    return convertor.nativeUnitsToVelocity(getTurnVelocityUPS());
  }

  public Double getAuxPosition() {
    return (_hasAuxController)
        ? _auxController.getSelectedSensorPosition(PID_PRIMARY)
        : getPosition();
  }

  public Double getAuxDistance() {
    return convertor.nativeUnitsToDistanceMeters(getAuxPosition());
  }

  public Double getAuxVelocityUPS() {
    return (_hasAuxController)
        ? _auxController.getSelectedSensorVelocity(PID_PRIMARY)
        : getVelocityUPS();
  }

  public Double getAuxVelocity() {
    return convertor.nativeUnitsToDistanceMeters(getAuxVelocityUPS());
  }

  public void setWithFF(final double pctOutput, final double arbFF) {
    _mode = SetPointMode.None;
    _controller.set(ControlMode.PercentOutput, pctOutput, DemandType.ArbitraryFeedForward, arbFF);
    if (_hasAuxController) {
      _auxController.follow(_controller);
    }
  }

  public void set(final double pctOutput) {
    this.setWithFF(pctOutput, 0.0);
  }

  public void setOutput(final double pctOutput, final double arbFF) {
    _mode = SetPointMode.None;
    _controller.set(ControlMode.PercentOutput, pctOutput);
  }

  public void setAuxOutput(final double auxOutput, final double arbFF) {
    _mode = SetPointMode.None;
    if (_hasAuxController) {
      _auxController.set(
          ControlMode.PercentOutput, auxOutput, DemandType.ArbitraryFeedForward, arbFF);
    }
  }

  public void setSeparateOutput(
      final double pctOutput, final double auxOutput, final double arbFF) {
    this.setOutput(pctOutput, arbFF);
    this.setAuxOutput(auxOutput, arbFF);
  }

  public void setSeparateOutput(final double pctOutput, final double auxOutput) {
    this.setSeparateOutput(pctOutput, auxOutput, 0.0);
  }

  public void setSeparateTarget(
      final double meters, final double auxMeters, final double _feedfwd) {
    _mode = SetPointMode.None;
    _setpoint = meters;
    _nativeSetpoint = convertor.distanceMetersToNativeUnits(_setpoint);
    _auxpoint = auxMeters;
    _nativeAuxpoint = convertor.distanceMetersToNativeUnits(_auxpoint);

    _controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    _controller.set(
        ControlMode.MotionMagic, _nativeSetpoint, DemandType.ArbitraryFeedForward, _feedfwd);
    if (_hasAuxController) {
      _auxController.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
      _auxController.set(
          ControlMode.MotionMagic, _nativeAuxpoint, DemandType.ArbitraryFeedForward, _feedfwd);
    }
    _mode = SetPointMode.Distance;
    Shuffleboard.addEventMarker("setTarget", name, EventImportance.kHigh);
    // System.out.println(name + " setTarget - " + _nativeSetpoint + " aux: " + _nativeAuxpoint + "
    // ffwd: " + _feedfwd);
  }

  public void setTargetWithFF(final double meters, final double _feedfwd) {
    _mode = SetPointMode.None;
    _setpoint = meters;
    _nativeSetpoint = convertor.distanceMetersToNativeUnits(_setpoint);
    _auxpoint = meters;
    _nativeAuxpoint = convertor.distanceMetersToNativeUnits(_auxpoint);

    _controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    _controller.set(
        ControlMode.MotionMagic, _nativeSetpoint, DemandType.ArbitraryFeedForward, _feedfwd);
    if (_hasAuxController) {
      _auxController.follow(_controller);
    }
    _mode = SetPointMode.DistanceFollow;
    Shuffleboard.addEventMarker("setTarget", name, EventImportance.kHigh);
    // System.out.println(name + " setTarget - " + _nativeSetpoint + " ffwd: " + _feedfwd);
  }

  public void setTarget(double meters) {
    setTargetWithFF(meters, 0.0);
  }

  public void setTargetAndAngle(final double meters, final double angle) {
    _mode = SetPointMode.None;
    _setpoint = meters;
    _nativeSetpoint = convertor.distanceMetersToNativeUnits(meters);
    _auxpoint = angle;
    _nativeAuxpoint = angle * 20.0; // 3600 for 180 degrees

    _controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    _controller.selectProfileSlot(kSlot_Turning, PID_TURN);
    _controller.set(ControlMode.MotionMagic, _nativeSetpoint, DemandType.AuxPID, _nativeAuxpoint);
    if (_hasAuxController) {
      _auxController.follow(_controller, FollowerType.AuxOutput1);
    }
    _mode = SetPointMode.DistanceAux;
    Shuffleboard.addEventMarker("setTargetAndAngle", name, EventImportance.kHigh);
    // System.out.println(name + " setTargetAndAngle - " + _nativeSetpoint);
  }

  public void setVelocity(final double velocity) {
    _mode = SetPointMode.None;
    _setpoint = velocity;
    _nativeSetpoint = convertor.velocityToNativeUnits(_setpoint);
    _auxpoint = _feedForwardCalculator.calculate(velocity);
    _nativeAuxpoint = _auxpoint / 12.;

    _controller.selectProfileSlot(kSlot_Velocit, PID_PRIMARY);
    _controller.set(
        ControlMode.Velocity, _nativeSetpoint, DemandType.ArbitraryFeedForward, _nativeAuxpoint);

    if (_hasAuxController) _auxController.follow(_controller);
    _mode = SetPointMode.Velocity;
    Shuffleboard.addEventMarker("setVelocity", name, EventImportance.kHigh);
  }

  public boolean isFwdLimitSwitchClosed() {
    return _controller.isFwdLimitSwitchClosed() == 1;
  }

  public boolean isRevLimitSwitchClosed() {
    return _controller.isRevLimitSwitchClosed() == 1;
  }

  public boolean isAuxFwdLimitSwitchClosed() {
    return (_hasAuxController)
        ? _auxController.isFwdLimitSwitchClosed() == 1
        : isFwdLimitSwitchClosed();
  }

  public boolean isAuxRevLimitSwitchClosed() {
    return (_hasAuxController)
        ? _auxController.isRevLimitSwitchClosed() == 1
        : isRevLimitSwitchClosed();
  }

  /** Resets the position of the Talon to 0. */
  public void resetPosition() {
    resetSensorPosition();
    resetAuxSensorPosition();
  }

  public void resetSensorPosition() {
    if (_controllerType == kTalonFX) {
      ((WPI_TalonFX) _controller).getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
    } else if (_controllerType == kTalonSRX) {
      ((WPI_TalonSRX) _controller).getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    } else {
      _controller.setSelectedSensorPosition(0, PID_PRIMARY, kTimeoutMs);
    }
  }

  public void resetAuxSensorPosition() {
    if (_hasAuxController) {
      if (_controllerType == kTalonFX) {
        ((WPI_TalonFX) _auxController)
            .getSensorCollection()
            .setIntegratedSensorPosition(0, kTimeoutMs);
      } else if (_controllerType == kTalonSRX) {
        ((WPI_TalonSRX) _auxController).getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
      } else {
        _auxController.setSelectedSensorPosition(0, PID_PRIMARY, kTimeoutMs);
      }
    }
  }

  public void configClearPositionOnLimitF(boolean enable) {
    _controller.configClearPositionOnLimitF(enable, kTimeoutMs);
    if (_hasAuxController) {
      _auxController.configClearPositionOnLimitF(enable, kTimeoutMs);
    }
  }

  public void configClearPositionOnLimitR(boolean enable) {
    _controller.configClearPositionOnLimitR(enable, kTimeoutMs);
    if (_hasAuxController) {
      _auxController.configClearPositionOnLimitR(enable, kTimeoutMs);
    }
  }

  public void configForwardSoftLimitThreshold(double thresh) {
    _controller.configForwardSoftLimitThreshold(thresh, kTimeoutMs);
    if (_hasAuxController) {
      _auxController.configForwardSoftLimitThreshold(thresh, kTimeoutMs);
    }
  }

  public void configReverseSoftLimitThreshold(double thresh) {
    _controller.configReverseSoftLimitThreshold(thresh, kTimeoutMs);
    if (_hasAuxController) {
      _auxController.configReverseSoftLimitThreshold(thresh, kTimeoutMs);
    }
  }

  public void configForwardSoftLimitEnable(boolean enable) {
    _controller.configForwardSoftLimitEnable(enable, kTimeoutMs);
    if (_hasAuxController) {
      _auxController.configForwardSoftLimitEnable(enable, kTimeoutMs);
    }
  }

  public void configReverseSoftLimitEnable(boolean enable) {
    _controller.configReverseSoftLimitEnable(enable, kTimeoutMs);
    if (_hasAuxController) {
      _auxController.configReverseSoftLimitEnable(enable, kTimeoutMs);
    }
  }

  public void overrideLimitSwitchesEnable(boolean enable) {
    _controller.overrideLimitSwitchesEnable(enable);
    if (_hasAuxController) {
      _auxController.overrideLimitSwitchesEnable(enable);
    }
  }

  public void enableBrakes(final boolean enabled) {
    _controller.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    if (_hasAuxController) {
      _auxController.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    }
  }

  public boolean isTargetFinished() {
    return _mode == SetPointMode.Finished;
  }

  /** Updates all cached values with current ones. */
  public void update() {
    switch (_mode) {
      case Distance:
        {
          _currentTrajPos = _controller.getActiveTrajectoryPosition();
          // _currentTrajPos = _controller.getSelectedSensorPosition();
          _currentAuxTrajPos =
              _hasAuxController ? _auxController.getActiveTrajectoryPosition() : _nativeAuxpoint;
          // _hasAuxController ? _auxController.getSelectedSensorPosition() : _nativeAuxpoint;

          if ((Math.abs(_currentTrajPos - _nativeSetpoint) < 100)
              && (Math.abs(_currentAuxTrajPos - _nativeAuxpoint) < 100)) {
            _mode = SetPointMode.Finished;
          }
          break;
        }
      case DistanceFollow:
      case DistanceAux:
        {
          _currentTrajPos = _controller.getActiveTrajectoryPosition();
          if (Math.abs(_currentTrajPos - _nativeSetpoint) < 10) {
            _mode = SetPointMode.Finished;
          }
          break;
        }
      case Velocity:
        {
          _currentTrajPos = _controller.getActiveTrajectoryVelocity();
          if (Math.abs(_currentTrajPos - _nativeSetpoint) < 10) {
            _mode = SetPointMode.Finished;
          }
          _controller.getFaults(_faults);
          if (_faults.SensorOutOfPhase) {
            Shuffleboard.addEventMarker("sensor is out of phase: ", name, EventImportance.kHigh);
            // System.out.println(
            //     "sensor is out of phase: " + _controller.getSelectedSensorVelocity(0));
          }
          break;
        }
      default:
        break;
    }
  }

  public void logPeriodic() {
    SmartDashboard.putString(name + "- Mode", _mode.toString());

    SmartDashboard.putNumber(name + "- SetPoint", _nativeSetpoint);
    SmartDashboard.putNumber(name + "- SetPointDistance", _setpoint);
    SmartDashboard.putNumber(name + "- Position", getPosition());
    SmartDashboard.putNumber(name + "- Distance", getDistance());
    SmartDashboard.putNumber(name + "- ActiveTrajectory", _currentTrajPos);
    SmartDashboard.putBoolean(name + "- FwdLimit", isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean(name + "- RevLimit", isRevLimitSwitchClosed());

    SmartDashboard.putString(name + "- ControlMode", _controller.getControlMode().toString());
    if (_controller.getControlMode() == ControlMode.MotionMagic) {
      SmartDashboard.putNumber(name + "- ClosedLoopError", _controller.getClosedLoopError());
      SmartDashboard.putNumber(name + "- ClosedLoopTgt", _controller.getClosedLoopTarget());
    }
    if (_hasAuxController) {
      SmartDashboard.putNumber(name + "- AuxPoint", _nativeAuxpoint);
      SmartDashboard.putNumber(name + "- AuxPointDistance", _auxpoint);
      SmartDashboard.putNumber(name + "- AuxPosition", getAuxPosition());
      SmartDashboard.putNumber(name + "- AuxDistance", getAuxDistance());
      SmartDashboard.putNumber(name + "- ActiveAuxTrajectory", _currentAuxTrajPos);
      SmartDashboard.putBoolean(name + "- AuxFwdLimit", isAuxFwdLimitSwitchClosed());
      SmartDashboard.putBoolean(name + "- AuxRevLimit", isAuxRevLimitSwitchClosed());

      SmartDashboard.putString(
          name + "- AuxControlMode", _auxController.getControlMode().toString());
      if (_auxController.getControlMode() == ControlMode.MotionMagic) {
        SmartDashboard.putNumber(
            name + "- AuxClosedLoopError", _auxController.getClosedLoopError());
        SmartDashboard.putNumber(name + "- AuxClosedLoopTgt", _auxController.getClosedLoopTarget());
      }
    }
  }
}
