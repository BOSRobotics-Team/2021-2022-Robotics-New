package frc.robot.util;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;

public class SmartMotor {
  /** ---- Flat constants, you should not need to change these ---- */

  /* BaseTalon types */
  public enum ControllerTypes {
    TalonNone {
      @Override
      public int value() {
        return 0;
      }
    },
    TalonSRX {
      @Override
      public int value() {
        return 1;
      }
    },
    TalonFX {
      @Override
      public int value() {
        return 2;
      }
    };

    public abstract int value();
  };

  /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
  public static final int REMOTE_0 = 0;
  public static final int REMOTE_1 = 1;

  /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
  public static final int PID_PRIMARY = 0;
  public static final int PID_AUX = 1;

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
      new GearRatios(Constants.kDriveGearRatio, Constants.kDriveWheelRadiusInches);
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
  private static final boolean kDebugLogging = false;

  public final Convertor m_convertor;

  private final ControllerTypes m_controllerType;
  private final BaseTalon m_controller;
  private final Faults m_faults = new Faults();

  private String m_name = "";
  private SimpleMotorFeedforward m_feedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);
  private FeedbackDevice m_feedbackDevice = FeedbackDevice.None;

  private enum SetPointMode {
    None,
    Distance,
    DistanceAux,
    Velocity,
    Finished
  };

  private SetPointMode _mode = SetPointMode.None;
  private double _setpoint = 0.0; // The most recently set setpoint.
  private double _nativeSetpoint = 0.0; // The setpoint in native units.
  private double _auxpoint = 0.0; // The most recently set auxpoint.
  private double _nativeAuxpoint = 0.0; // The auxpoint in native units.
  private double _feedFwd = 0.0; // The feed forward factor.
  private double _currentTrajPos = 0.0; // The current trajectory position.
  private double _currentAuxTrajPos = 0.0; // The current aux trajectory position.

  public SmartMotor(final BaseTalon talon, final String nm) {
    if (talon.getClass() == WPI_TalonFX.class) {
      m_controllerType = ControllerTypes.TalonFX;
    } else if (talon.getClass() == WPI_TalonSRX.class) {
      m_controllerType = ControllerTypes.TalonSRX;
    } else {
      m_controllerType = ControllerTypes.TalonNone;
    }
    m_convertor = new Convertor(kSensorUnitsPerRotation[m_controllerType.value()]);
    m_convertor.setRatios(kDefaultGearRatio);

    m_controller = talon;
    m_controller.configFactoryDefault();

    this.setName(nm);
    this.setFeedbackDevice(kDefaultFeedbackDevice[m_controllerType.value()]);
  }

  public SmartMotor(final BaseTalon talon) {
    this(talon, "Device " + talon.getDeviceID());
  }

  public BaseTalon getController() {
    return m_controller;
  }

  public Convertor getConvertor() {
    return m_convertor;
  }

  public int getDeviceID() {
    return m_controller.getDeviceID();
  }

  public ControllerTypes getControllerType() {
    return m_controllerType;
  }

  public TalonFXSimCollection getFXSimCollection() {
    return (m_controllerType == ControllerTypes.TalonFX)
        ? ((TalonFX) m_controller).getSimCollection()
        : null;
  }

  public TalonSRXSimCollection getSRXSimCollection() {
    return (m_controllerType == ControllerTypes.TalonSRX)
        ? ((TalonSRX) m_controller).getSimCollection()
        : null;
  }

  public void initController() {
    _mode = SetPointMode.None;
    /* Configure Sensor Source for Primary PID */
    m_controller.configSelectedFeedbackSensor(m_feedbackDevice, PID_PRIMARY, kTimeoutMs);

    /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
    m_controller.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

    // Set the peak and nominal outputs
    m_controller.configNominalOutputForward(0, kTimeoutMs);
    m_controller.configNominalOutputReverse(0, kTimeoutMs);
    m_controller.configPeakOutputForward(1, kTimeoutMs);
    m_controller.configPeakOutputReverse(-1, kTimeoutMs);

    // Set limit switch devices
    if (m_controllerType == ControllerTypes.TalonFX) {
      m_controller.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, kTimeoutMs);
      m_controller.configReverseLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, kTimeoutMs);
    }
    // Clear the reset and soft limits
    m_controller.configClearPositionOnLimitF(false, kTimeoutMs);
    m_controller.configClearPositionOnLimitR(false, kTimeoutMs);
    m_controller.configForwardSoftLimitEnable(false, kTimeoutMs);
    m_controller.configReverseSoftLimitEnable(false, kTimeoutMs);

    // Set default PID values
    this.setDistanceConfigs(SmartMotor.kDefaultGains_Distanc);
  }

  public void setName(String nm) {
    m_name = nm;
  }

  public void setInverted(boolean invert) {
    m_controller.setInverted(invert);
  }

  public void setInverted(InvertType invertType) {
    m_controller.setInverted(invertType);
  }

  public void setFeedbackDevice(FeedbackDevice fbDev) {
    m_feedbackDevice = fbDev;
  }

  public void configureRatios(final GearRatios gearRatio) {
    m_convertor.setRatios(gearRatio);
  }

  public void configureFeedForward(final double gain, final double velGain) {
    m_feedForwardCalculator = new SimpleMotorFeedforward(gain, velGain);
  }

  public void configureFeedForward(final double gain) {
    configureFeedForward(gain, 0);
  }

  public void setClosedLoopGains(final int slot, final Gains gain) {
    m_controller.config_kP(slot, gain.kP, kTimeoutMs);
    m_controller.config_kI(slot, gain.kI, kTimeoutMs);
    m_controller.config_kD(slot, gain.kD, kTimeoutMs);
    m_controller.config_kF(slot, gain.kF, kTimeoutMs);
    m_controller.config_IntegralZone(slot, gain.kIzone, kTimeoutMs);
    m_controller.configClosedLoopPeakOutput(slot, gain.kPeakOutput);
    m_controller.configAllowableClosedloopError(slot, 0, kTimeoutMs);
    m_controller.configClosedLoopPeriod(slot, 1);
  }

  public void setDistanceConfigs(final Gains gains) {
    this.setClosedLoopGains(kSlot_Distanc, gains);

    // Set acceleration and vcruise velocity - see documentation
    m_controller.configMotionAcceleration(gains.kMotionAccel, kTimeoutMs);
    m_controller.configMotionCruiseVelocity(gains.kMotionCruiseVel, kTimeoutMs);
    m_controller.configMotionSCurveStrength(gains.kMotionSCurve, kTimeoutMs);

    m_controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    m_controller.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
    m_controller.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, kTimeoutMs);
    m_controller.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, kTimeoutMs);

    /* Configure Sensor Source for Primary PID */
    m_controller.configSelectedFeedbackSensor(m_feedbackDevice, PID_PRIMARY, kTimeoutMs);
  }

  public void setDistanceConfigs(final Gains gains, int auxId) {
    this.setDistanceConfigs(gains);

    m_controller.configRemoteFeedbackFilter(
        auxId, RemoteSensorSource.TalonFX_SelectedSensor, REMOTE_0);

    /* Check if we're inverted */
    if (m_controller.getInverted()) {
      m_controller.configSensorTerm(SensorTerm.Diff0, m_feedbackDevice);
      m_controller.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0);
      m_controller.configSelectedFeedbackSensor(
          FeedbackDevice.SensorDifference, PID_PRIMARY, kTimeoutMs);
    } else {
      m_controller.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
      m_controller.configSensorTerm(SensorTerm.Sum1, m_feedbackDevice);
      m_controller.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_PRIMARY, kTimeoutMs);
    }
    m_controller.configAuxPIDPolarity(false, kTimeoutMs);
    m_controller.configSelectedFeedbackCoefficient(0.5, PID_PRIMARY, kTimeoutMs);
  }

  public void setDistanceAndTurnConfigs(final Gains dgains, final Gains tgains, int auxId) {
    this.setDistanceConfigs(dgains, auxId);

    this.setClosedLoopGains(kSlot_Turning, tgains);
    m_controller.selectProfileSlot(kSlot_Turning, PID_AUX);
    m_controller.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, kTimeoutMs);
    m_controller.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10, kTimeoutMs);

    /* Check if we're inverted */
    if (m_controller.getInverted()) {
      m_controller.configSensorTerm(SensorTerm.Sum0, m_feedbackDevice);
      m_controller.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor0);
      m_controller.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_AUX, kTimeoutMs);
      m_controller.configAuxPIDPolarity(true, kTimeoutMs);
    } else {
      m_controller.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0);
      m_controller.configSensorTerm(SensorTerm.Diff1, m_feedbackDevice);
      m_controller.configSelectedFeedbackSensor(
          FeedbackDevice.SensorDifference, PID_AUX, kTimeoutMs);
      m_controller.configAuxPIDPolarity(true, kTimeoutMs);
    }
    m_controller.configSelectedFeedbackCoefficient(kFeedbackCoefficient, PID_AUX, kTimeoutMs);
  }

  public void setVelocityConfigs(final Gains gains) {
    this.setClosedLoopGains(kSlot_Velocit, gains);

    m_controller.selectProfileSlot(kSlot_Velocit, PID_PRIMARY);
    m_controller.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
    m_controller.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10);
    m_controller.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, kTimeoutMs);

    /* Configure Sensor Source for Primary PID */
    m_controller.configSelectedFeedbackSensor(m_feedbackDevice, PID_PRIMARY, kTimeoutMs);
  }

  /**
   * Get the position of the drive.
   *
   * @return The signed position in units, or null if the drive doesn't have encoders.
   */
  public Double getPosition() {
    return m_controller.getSelectedSensorPosition(PID_PRIMARY);
  }

  /**
   * Get the position of the drive in meters.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  public Double getDistance() {
    return m_convertor.nativeUnitsToDistanceMeters(getPosition());
  }

  /**
   * Get the aux position of the drive.
   *
   * @return The signed position in units, or null if the drive doesn't have encoders.
   */
  public Double getTurnPosition() {
    return m_controller.getSelectedSensorPosition(PID_AUX);
  }

  /**
   * Get the aux position of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  public Double getTurnDistance() {
    return m_convertor.nativeUnitsToDistanceMeters(getTurnPosition());
  }

  /**
   * Get the velocity of the drive.
   *
   * @return The signed velocity in units per second, or null if the drive doesn't have encoders.
   */
  public Double getNativeVelocity() {
    return m_controller.getSelectedSensorVelocity(PID_PRIMARY);
  }

  /**
   * Get the velocity of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  public Double getVelocity() {
    return m_convertor.nativeUnitsToVelocity(getNativeVelocity());
  }

  /**
   * Get the angular velocity of the drive.
   *
   * @return The signed velocity in units per second, or null if the drive doesn't have encoders.
   */
  public Double getNativeTurnVelocity() {
    return m_controller.getSelectedSensorVelocity(PID_AUX);
  }

  /**
   * Get the angular velocity of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  public Double getTurnVelocity() {
    return m_convertor.nativeUnitsToVelocity(getNativeTurnVelocity());
  }

  public void follow(IMotorController masterCtrl) {
    m_controller.follow(masterCtrl);
  }

  public void follow(IMotorController masterCtrl, FollowerType type) {
    m_controller.follow(masterCtrl, type);
  }

  public void set(final double pctOutput, final double arbFF) {
    _mode = SetPointMode.None;
    m_controller.set(ControlMode.PercentOutput, pctOutput, DemandType.ArbitraryFeedForward, arbFF);
  }

  public void set(final double pctOutput) {
    this.set(pctOutput, 0.0);
  }

  public void setTargetWithFF(final double meters, final double feedfwd) {
    _mode = SetPointMode.None;
    _setpoint = meters;
    _nativeSetpoint = m_convertor.distanceMetersToNativeUnits(_setpoint);
    _auxpoint = _nativeAuxpoint = 0.0;
    _feedFwd = feedfwd;

    m_controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    m_controller.set(
        ControlMode.MotionMagic, _nativeSetpoint, DemandType.ArbitraryFeedForward, _feedFwd);

    _mode = SetPointMode.Distance;
    Shuffleboard.addEventMarker("setTarget", m_name, EventImportance.kHigh);
    // System.out.println(name + " setTarget - " + _nativeSetpoint + " ffwd: " + _feedFwd);
  }

  public void setTargetWithFF(final double meters, final double feedfwd, final BaseTalon talon) {
    this.setTargetWithFF(meters, 0.0);
    talon.follow(m_controller);
  }

  public void setTargetWithFF(final double meters, final double feedfwd, final SmartMotor motor) {
    this.setTargetWithFF(meters, 0.0);
    motor.follow(m_controller);
  }

  public void setTarget(final double meters) {
    this.setTargetWithFF(meters, 0.0);
  }

  public void setTarget(final double meters, final BaseTalon talon) {
    this.setTargetWithFF(meters, 0.0, talon);
  }

  public void setTarget(final double meters, final SmartMotor motor) {
    this.setTargetWithFF(meters, 0.0, motor);
  }

  public void setTargetAndAngle(final double meters, final double angle) {
    _mode = SetPointMode.None;
    _setpoint = meters;
    _nativeSetpoint = m_convertor.distanceMetersToNativeUnits(meters);
    _auxpoint = angle;
    _nativeAuxpoint = _auxpoint * 20.0; // 3600 for 180 degrees
    _feedFwd = 0.0;

    m_controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    m_controller.selectProfileSlot(kSlot_Turning, PID_AUX);
    m_controller.set(ControlMode.MotionMagic, _nativeSetpoint, DemandType.AuxPID, _nativeAuxpoint);

    _mode = SetPointMode.DistanceAux;
    Shuffleboard.addEventMarker("setTargetAndAngle", m_name, EventImportance.kHigh);
    // System.out.println(name + " setTargetAndAngle - " + _nativeSetpoint);
  }

  public void setTargetAndAngle(final double meters, final double angle, final BaseTalon talon) {
    this.setTargetAndAngle(meters, angle);
    talon.follow(m_controller, FollowerType.AuxOutput1);
  }

  public void setTargetAndAngle(final double meters, final double angle, final SmartMotor motor) {
    this.setTargetAndAngle(meters, angle);
    motor.follow(m_controller, FollowerType.AuxOutput1);
  }

  public void setVelocity(final double velocity) {
    _mode = SetPointMode.None;
    _setpoint = velocity;
    _nativeSetpoint = m_convertor.velocityToNativeUnits(_setpoint);
    _auxpoint = _nativeAuxpoint = 0.0;
    _feedFwd = m_feedForwardCalculator.calculate(velocity) / 12.0;

    m_controller.selectProfileSlot(kSlot_Velocit, PID_PRIMARY);
    m_controller.set(
        ControlMode.Velocity, _nativeSetpoint, DemandType.ArbitraryFeedForward, _feedFwd);

    _mode = SetPointMode.Velocity;
    Shuffleboard.addEventMarker("setVelocity", m_name, EventImportance.kHigh);
  }

  public void enableBrakes(final boolean enabled) {
    m_controller.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public boolean isTargetFinished() {
    return _mode == SetPointMode.Finished;
  }

  public void resetSensorPosition() {
    switch (m_controllerType) {
      case TalonFX:
        {
          ((WPI_TalonFX) m_controller)
              .getSensorCollection()
              .setIntegratedSensorPosition(0, kTimeoutMs);
          break;
        }
      case TalonSRX:
        {
          ((WPI_TalonSRX) m_controller).getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
          break;
        }
      default:
        {
          m_controller.setSelectedSensorPosition(0, PID_PRIMARY, kTimeoutMs);
          break;
        }
    }
  }

  public boolean isFwdLimitSwitchClosed() {
    return m_controller.isFwdLimitSwitchClosed() == 1;
  }

  public boolean isRevLimitSwitchClosed() {
    return m_controller.isRevLimitSwitchClosed() == 1;
  }

  public void configClearPositionOnLimitF(boolean enable) {
    m_controller.configClearPositionOnLimitF(enable, kTimeoutMs);
  }

  public void configClearPositionOnLimitR(boolean enable) {
    m_controller.configClearPositionOnLimitR(enable, kTimeoutMs);
  }

  public void configForwardSoftLimitThreshold(double thresh) {
    m_controller.configForwardSoftLimitThreshold(thresh, kTimeoutMs);
  }

  public void configReverseSoftLimitThreshold(double thresh) {
    m_controller.configReverseSoftLimitThreshold(thresh, kTimeoutMs);
  }

  public void configForwardSoftLimitDistance(double meters) {
    m_controller.configForwardSoftLimitThreshold(
        m_convertor.distanceMetersToNativeUnits(meters), kTimeoutMs);
  }

  public void configReverseSoftLimitDistance(double meters) {
    m_controller.configReverseSoftLimitThreshold(
        m_convertor.distanceMetersToNativeUnits(meters), kTimeoutMs);
  }

  public void configForwardSoftLimitEnable(boolean enable) {
    m_controller.configForwardSoftLimitEnable(enable, kTimeoutMs);
  }

  public void configReverseSoftLimitEnable(boolean enable) {
    m_controller.configReverseSoftLimitEnable(enable, kTimeoutMs);
  }

  public void overrideLimitSwitchesEnable(boolean enable) {
    m_controller.overrideLimitSwitchesEnable(enable);
  }

  /** Updates all cached values with current ones. */
  public void update() {
    switch (_mode) {
      case DistanceAux:
        {
          _currentTrajPos = m_controller.getActiveTrajectoryPosition(PID_PRIMARY);
          _currentAuxTrajPos = m_controller.getActiveTrajectoryPosition(PID_AUX);

          if ((Math.abs(_currentTrajPos - _nativeSetpoint) < 100)
              && (Math.abs(_currentAuxTrajPos - _nativeAuxpoint) < 100)) {
            _mode = SetPointMode.Finished;
          }
          break;
        }
      case Distance:
        {
          _currentTrajPos = m_controller.getActiveTrajectoryPosition(PID_PRIMARY);
          if (Math.abs(_currentTrajPos - _nativeSetpoint) < 100) {
            _mode = SetPointMode.Finished;
          }
          break;
        }
      case Velocity:
        {
          _currentTrajPos = m_controller.getActiveTrajectoryVelocity(PID_PRIMARY);
          if (Math.abs(_currentTrajPos - _nativeSetpoint) < 100) {
            _mode = SetPointMode.Finished;
          }
          m_controller.getFaults(m_faults);
          if (m_faults.SensorOutOfPhase) {
            Shuffleboard.addEventMarker("sensor is out of phase: ", m_name, EventImportance.kHigh);
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
    SmartDashboard.putString(m_name + "- Mode", _mode.toString());

    SmartDashboard.putNumber(m_name + "- SetPoint", _nativeSetpoint);
    SmartDashboard.putNumber(m_name + "- SetPointDistance", _setpoint);
    SmartDashboard.putNumber(m_name + "- FeedForward", _feedFwd);
    SmartDashboard.putNumber(m_name + "- ActiveTrajectory", _currentTrajPos);

    if (kDebugLogging) {
      SmartDashboard.putNumber(m_name + "- Position", getPosition());
      SmartDashboard.putNumber(m_name + "- Distance", getDistance());
      SmartDashboard.putBoolean(m_name + "- FwdLimit", isFwdLimitSwitchClosed());
      SmartDashboard.putBoolean(m_name + "- RevLimit", isRevLimitSwitchClosed());

      SmartDashboard.putString(m_name + "- ControlMode", m_controller.getControlMode().toString());
      if (m_controller.getControlMode() == ControlMode.MotionMagic) {
        SmartDashboard.putNumber(m_name + "- ClosedLoopError", m_controller.getClosedLoopError());
        SmartDashboard.putNumber(m_name + "- ClosedLoopTgt", m_controller.getClosedLoopTarget());
      }
    }
  }
}
