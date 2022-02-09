package frc.robot.util;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

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
      new GearRatios(Constants.kGearRatio, Constants.kWheelRadiusInches, 1.0);
  public static final Gains kDefaultGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.80);

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

  private final BaseTalon _controller;
  private final BaseTalon _auxController;

  private final int _controllerType;
  private final Convertor _convertor;
  private final Faults _faults = new Faults();
  private final SimpleMotorFeedforward _feedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);

  private FeedbackDevice _feedbackDevice = FeedbackDevice.None;

  private enum SetPointMode {
    None,
    Distance,
    DistanceAux,
    Velocity,
    Finished
  };

  private SetPointMode _mode = SetPointMode.None;
  private double _setpoint = 0.0; // The most recently set setpoint.
  private double _nativeSetpoint =
      0.0; // The setpoint in native units. Field to avoid garbage collection.
  private double _auxpoint = 0.0; // The most recently set setpoint.
  private double _nativeAuxpoint =
      0.0; // The setpoint in native units. Field to avoid garbage collection.

  public SmartMotorController(final BaseTalon talon, final BaseTalon auxTalon) {
    _controller = talon;
    _auxController = auxTalon;

    _controller.configFactoryDefault();
    if (_controller.getClass() == WPI_TalonFX.class) {
      _controllerType = kTalonFX;
      if (RobotBase.isSimulation())
        PhysicsSim.getInstance().addTalonFX((WPI_TalonFX) _controller, 0.2, 6800);
    } else if (_controller.getClass() == WPI_TalonSRX.class) {
      _controllerType = kTalonSRX;
      if (RobotBase.isSimulation())
        PhysicsSim.getInstance().addTalonSRX((WPI_TalonSRX) _controller, 0.2, 6800);
    } else {
      _controllerType = kTalonNone;
    }
    _feedbackDevice = kDefaultFeedbackDevice[_controllerType];

    _convertor = new Convertor(kSensorUnitsPerRotation[_controllerType]);
    _convertor.setRatios(kDefaultGearRatio);

    if (_auxController != null) {
      _auxController.configFactoryDefault();
      if (RobotBase.isSimulation()) {
        if (_auxController.getClass() == WPI_TalonFX.class) {
          PhysicsSim.getInstance().addTalonFX((WPI_TalonFX) _auxController, 0.2, 6800);
        } else if (_auxController.getClass() == WPI_TalonSRX.class) {
          PhysicsSim.getInstance().addTalonSRX((WPI_TalonSRX) _auxController, 0.2, 6800);
        }
      }
    }
  }

  public SmartMotorController(final BaseTalon talon) {
    this(talon, null);
  }

  public void initController() {
    _mode = SetPointMode.None;
    initController(_controller);
    if (_auxController != null) initController(_auxController);
  }

  public void configureRatios(GearRatios gearRatio) {
    _convertor.setRatios(gearRatio);
  }

  private void initController(BaseTalon talon) {
    /* Configure Sensor Source for Primary PID */
    talon.configSelectedFeedbackSensor(_feedbackDevice, PID_PRIMARY, kTimeoutMs);

    /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
    talon.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

    // _controller.setSensorPhase(false);
    talon.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    talon.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);

    /* Set the peak and nominal outputs */
    talon.configNominalOutputForward(0, kTimeoutMs);
    talon.configNominalOutputReverse(0, kTimeoutMs);
    talon.configPeakOutputForward(1, kTimeoutMs);
    talon.configPeakOutputReverse(-1, kTimeoutMs);

    talon.configMotionAcceleration(6000, kTimeoutMs);
    talon.configMotionCruiseVelocity(15000, kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    setClosedLoopGains(talon, kSlot_Distanc, kDefaultGains_Distanc);
    talon.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
  }

  private void setClosedLoopGains(BaseTalon talon, int slot, Gains gain) {
    talon.config_kP(slot, gain.kP, kTimeoutMs);
    talon.config_kI(slot, gain.kI, kTimeoutMs);
    talon.config_kD(slot, gain.kD, kTimeoutMs);
    talon.config_kF(slot, gain.kF, kTimeoutMs);
    talon.config_IntegralZone(slot, gain.kIzone, kTimeoutMs);
    talon.configClosedLoopPeakOutput(slot, gain.kPeakOutput);
    talon.configAllowableClosedloopError(slot, 0, kTimeoutMs);
    //		talon.configMaxIntegralAccumulator(slot, gain. maxIntegral, kTimeoutMs);

    //      Set acceleration and vcruise velocity - see documentation
    talon.configClosedLoopPeriod(slot, 1);
  }

  public void setDistanceConfigs(Gains gains) {
    setClosedLoopGains(_controller, kSlot_Distanc, gains);

    _controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);

    /* Configure Sensor Source for Primary PID */
    _controller.configSelectedFeedbackSensor(_feedbackDevice, PID_PRIMARY, kTimeoutMs);

    if (_auxController != null) {
      setClosedLoopGains(_auxController, kSlot_Distanc, gains);
      _auxController.configRemoteFeedbackFilter(
          _controller.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, REMOTE_0);
      /* Check if we're inverted */
      if (_auxController.getInverted()) {
        _auxController.configSensorTerm(SensorTerm.Diff0, _feedbackDevice);
        _auxController.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0);
        _auxController.configSelectedFeedbackSensor(
            FeedbackDevice.SensorDifference, PID_PRIMARY, kTimeoutMs);
      } else {
        _auxController.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
        _auxController.configSensorTerm(SensorTerm.Sum1, _feedbackDevice);
        _auxController.configSelectedFeedbackSensor(
            FeedbackDevice.SensorSum, PID_PRIMARY, kTimeoutMs);
      }
      _auxController.configSelectedFeedbackCoefficient(0.5, PID_PRIMARY, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10, kTimeoutMs);
    }
    /* Set relevant frame periods to be at least as fast as periodic rate */
    _controller.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, kTimeoutMs);
  }

  public void setDistanceAndTurnConfigs(Gains dgains, Gains tgains) {
    setDistanceConfigs(dgains);

    setClosedLoopGains(_controller, kSlot_Turning, tgains);

    _controller.selectProfileSlot(kSlot_Turning, PID_TURN);

    if (_auxController != null) {
      setClosedLoopGains(_auxController, kSlot_Turning, tgains);
      _auxController.configRemoteFeedbackFilter(
          _controller.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, REMOTE_1);
      /* Check if we're inverted */
      if (_auxController.getInverted()) {
        _auxController.configSensorTerm(SensorTerm.Sum0, _feedbackDevice);
        _auxController.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor1);
        _auxController.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_TURN, kTimeoutMs);
        _auxController.configAuxPIDPolarity(true, kTimeoutMs);
      } else {
        _auxController.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor1);
        _auxController.configSensorTerm(SensorTerm.Diff1, _feedbackDevice);
        _auxController.configSelectedFeedbackSensor(
            FeedbackDevice.SensorDifference, PID_TURN, kTimeoutMs);
        _auxController.configAuxPIDPolarity(true, kTimeoutMs);
      }
      _auxController.configSelectedFeedbackCoefficient(kFeedbackCoefficient, PID_TURN, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10, kTimeoutMs);
    }
    /* Set relevant frame periods to be at least as fast as periodic rate */
    _controller.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10, kTimeoutMs);
  }

  public void setVelocityConfigs(Gains gains) {
    setClosedLoopGains(_controller, kSlot_Velocit, gains);

    _controller.selectProfileSlot(kSlot_Velocit, PID_PRIMARY);

    /* Configure Sensor Source for Primary PID */
    _controller.configSelectedFeedbackSensor(_feedbackDevice, PID_PRIMARY, kTimeoutMs);

    if (_auxController != null) {
      setClosedLoopGains(_auxController, kSlot_Velocit, gains);
      _auxController.configRemoteFeedbackFilter(
          _controller.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, REMOTE_0);
      /* Check if we're inverted */
      if (_auxController.getInverted()) {
        _auxController.configSensorTerm(SensorTerm.Diff0, _feedbackDevice);
        _auxController.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0);
        _auxController.configSelectedFeedbackSensor(
            FeedbackDevice.SensorDifference, PID_PRIMARY, kTimeoutMs);
      } else {
        _auxController.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
        _auxController.configSensorTerm(SensorTerm.Sum1, _feedbackDevice);
        _auxController.configSelectedFeedbackSensor(
            FeedbackDevice.SensorSum, PID_PRIMARY, kTimeoutMs);
      }
      _auxController.configSelectedFeedbackCoefficient(0.5, PID_PRIMARY, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10);
      _auxController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
      _auxController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, kTimeoutMs);
    }
    /* Set relevant frame periods to be at least as fast as periodic rate */
    _controller.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, kTimeoutMs);
    _controller.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, kTimeoutMs);
  }

  /**
   * Get the velocity of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  public Double getNativeVelocity() {
    return (_auxController != null)
        ? _auxController.getSelectedSensorVelocity(PID_PRIMARY)
        : _controller.getSelectedSensorVelocity(PID_PRIMARY);
  }

  public Double getVelocity() {
    return _convertor.nativeUnitsToVelocity(getNativeVelocity());
  }
  /**
   * Get the velocity of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  public Double getAuxNativeVelocity() {
    return (_auxController != null)
        ? _auxController.getSelectedSensorVelocity(PID_TURN)
        : _controller.getSelectedSensorVelocity(PID_TURN);
  }

  public Double getAuxVelocity() {
    return _convertor.nativeUnitsToVelocity(getAuxNativeVelocity());
  }

  /**
   * Get the position of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  public Double getNativePosition() {
    return (_auxController != null)
        ? _auxController.getSelectedSensorPosition(PID_PRIMARY)
        : _controller.getSelectedSensorPosition(PID_PRIMARY);
  }

  public Double getPosition() {
    return _convertor.nativeUnitsToDistanceMeters(getNativePosition());
  }

  public Double getLeftPosition() {
    return _controller.getSelectedSensorPosition(PID_PRIMARY);
  }

  public Double getLeftDistance() {
    return _convertor.nativeUnitsToDistanceMeters(getLeftPosition());
  }

  public Double getLeftVelocity() {
    return _convertor.nativeUnitsToDistanceMeters(
        _controller.getSelectedSensorVelocity(PID_PRIMARY));
  }

  public Double getRightPosition() {
    return getNativePosition();
  }

  public Double getRightDistance() {
    return getPosition();
  }

  public Double getRightVelocity() {
    return getVelocity();
  }

  /**
   * Get the position2 of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  public Double getAuxNativePosition() {
    return (_auxController != null)
        ? _auxController.getSelectedSensorPosition(PID_TURN)
        : _controller.getSelectedSensorPosition(PID_TURN);
  }

  public Double getAuxPosition() {
    return _convertor.nativeUnitsToDistanceMeters(getAuxNativePosition());
  }

  public void set(double pctOutput) {
    _mode = SetPointMode.None;
    _controller.set(ControlMode.PercentOutput, pctOutput);
    if (_auxController != null) _auxController.follow(_controller, FollowerType.PercentOutput);
  }

  public void set(double pctOutput, double auxOutput) {
    _mode = SetPointMode.None;
    _controller.set(ControlMode.PercentOutput, pctOutput);
    if (_auxController != null) _auxController.set(ControlMode.PercentOutput, auxOutput);
  }

  public void setTarget(double meters) {
    _mode = SetPointMode.None;
    _setpoint = meters;
    _nativeSetpoint = _convertor.distanceMetersToNativeUnits(meters);
    _auxpoint = _nativeAuxpoint = 0;

    _controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    _controller.selectProfileSlot(kSlot_Turning, PID_TURN);
    if (_auxController != null) {
      _controller.follow(_auxController, FollowerType.PercentOutput);

      _auxController.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
      _auxController.selectProfileSlot(kSlot_Turning, PID_TURN);
      _auxController.set(ControlMode.MotionMagic, _nativeSetpoint);
    } else {
      _controller.set(ControlMode.MotionMagic, _nativeSetpoint);
    }
    _mode = SetPointMode.Distance;
  }

  public void setTarget(double meters, double aux) {
    _mode = SetPointMode.None;
    _setpoint = meters;
    _nativeSetpoint = _convertor.distanceMetersToNativeUnits(meters);

    _auxpoint = aux;
    _nativeAuxpoint = aux * 20.0; // 3600 for 180 degrees

    _controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    _controller.selectProfileSlot(kSlot_Turning, PID_TURN);
    if (_auxController != null) {
      _controller.follow(_auxController, FollowerType.AuxOutput1);
      _auxController.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
      _auxController.selectProfileSlot(kSlot_Turning, PID_TURN);
      _auxController.set(
          ControlMode.MotionMagic, _nativeSetpoint, DemandType.AuxPID, _nativeAuxpoint);
    } else {
      _controller.set(ControlMode.MotionMagic, _nativeSetpoint, DemandType.AuxPID, _nativeAuxpoint);
    }
    _mode = SetPointMode.DistanceAux;
  }

  public void setVelocityUPS(final double velocity) {
    _mode = SetPointMode.None;
    _setpoint = velocity;
    _nativeSetpoint = _convertor.velocityToNativeUnits(_setpoint);
    _auxpoint = _feedForwardCalculator.calculate(velocity);
    _nativeAuxpoint = _auxpoint / 12.;

    _controller.selectProfileSlot(kSlot_Velocit, PID_PRIMARY);
    _controller.set(
        ControlMode.Velocity, _nativeSetpoint, DemandType.ArbitraryFeedForward, _nativeAuxpoint);
    if (_auxController != null) {
      _auxController.selectProfileSlot(kSlot_Velocit, PID_PRIMARY);
      _auxController.follow(_controller, FollowerType.PercentOutput);
    }
    _mode = SetPointMode.Velocity;
  }

  public boolean isFwdLimitSwitchClosed() {
    return _controller.isFwdLimitSwitchClosed() == 1;
  }

  public boolean isRevLimitSwitchClosed() {
    return _controller.isRevLimitSwitchClosed() == 1;
  }

  /** Resets the position of the Talon to 0. */
  public void resetPosition() {
    if (_controllerType == kTalonFX) {
      ((WPI_TalonFX) _controller).getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
      if (_auxController != null)
        ((WPI_TalonFX) _auxController)
            .getSensorCollection()
            .setIntegratedSensorPosition(0, kTimeoutMs);
    } else if (_controllerType == kTalonSRX) {
      ((WPI_TalonSRX) _controller).getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
      if (_auxController != null)
        ((WPI_TalonSRX) _auxController).getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    } else {
      _controller.setSelectedSensorPosition(0, PID_PRIMARY, kTimeoutMs);
      if (_auxController != null)
        _auxController.setSelectedSensorPosition(0, PID_PRIMARY, kTimeoutMs);
    }
  }

  public void enableBrakes(boolean enabled) {
    _controller.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    if (_auxController != null)
      _auxController.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public double getClosedLoopTarget() {
    return (_auxController != null)
        ? _auxController.getClosedLoopTarget()
        : _controller.getClosedLoopTarget();
  }

  public double getClosedLoopError() {
    return (_auxController != null)
        ? _auxController.getClosedLoopError()
        : _controller.getClosedLoopError();
  }

  public double getActiveTrajectoryPosition() {
    return (_auxController != null)
        ? _auxController.getActiveTrajectoryPosition()
        : _controller.getActiveTrajectoryPosition();
  }

  public double getActiveTrajectoryVelocity() {
    return (_auxController != null)
        ? _auxController.getActiveTrajectoryVelocity()
        : _controller.getActiveTrajectoryVelocity();
  }

  public boolean isTargetFinished() {
    return _mode == SetPointMode.Finished;
  }

  /** Updates all cached values with current ones. */
  public void update() {
    if (RobotBase.isSimulation()) {
      PhysicsSim.getInstance().run();
    }

    switch (_mode) {
      case Distance:
        {
          double trajPos = getActiveTrajectoryPosition();
          System.out.println(
              "SmartMotorController - targetPos: " + _nativeSetpoint + " pos: " + trajPos);
          if (Math.abs(trajPos - _nativeSetpoint) < 10) _mode = SetPointMode.Finished;
          break;
        }
      case DistanceAux:
        {
          double trajPos = getActiveTrajectoryPosition();
          double looptarget = getClosedLoopTarget();
          double looperror = getClosedLoopError();
          System.out.println(
              "SmartMotorController - targetPos: "
                  + _nativeSetpoint
                  + " pos: "
                  + trajPos
                  + " tgt: "
                  + looptarget
                  + " err: "
                  + looperror);
          if (Math.abs(trajPos - _nativeSetpoint) < 10) _mode = SetPointMode.Finished;
          break;
        }
      case Velocity:
        {
          double trajVel = getActiveTrajectoryVelocity();
          System.out.println(
              "SmartMotorController - targetPos: " + _nativeSetpoint + " pos: " + trajVel);
          if (Math.abs(trajVel - _nativeSetpoint) < 10) _mode = SetPointMode.Finished;

          _controller.getFaults(_faults);
          if (_faults.SensorOutOfPhase) {
            double leftVelUnitsPer100ms = _controller.getSelectedSensorVelocity(0);
            System.out.println("sensor is out of phase: " + leftVelUnitsPer100ms);
          }
          break;
        }
      default:
        break;
    }
  }
}
