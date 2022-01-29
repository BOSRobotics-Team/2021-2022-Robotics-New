package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SmartMotorHelper {
	/** ---- Flat constants, you should not need to change these ---- */
    public static final int kTalonNone = -1;
    public static final int kTalonSRX = 0;
    public static final int kTalonFX = 1;

	/* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
	public static final int REMOTE_0 = 0;
	public static final int REMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public static final int PID_PRIMARY = 0;
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

    public static final int kSensorUnitsPerRotation[] = { 4096, 2048 };
    public static final FeedbackDevice kDefaultFeedbackDevice[] = { FeedbackDevice.CTRE_MagEncoder_Relative, FeedbackDevice.IntegratedSensor };

    public static final GearRatios kDefaultGearRatio = new GearRatios(Constants.kGearRatio, Constants.kWheelRadiusInches, 1.0);
    public static final Gains kDefaultGains_Distanc = new Gains( 0.1, 0.0, 0.0, 0.0, 100, 0.80 );

    /**
	 * Empirically measure what the difference between encoders per 360'
	 * Drive the robot in clockwise rotations and measure the units per rotation.
	 * Drive the robot in counter clockwise rotations and measure the units per rotation.
	 * Take the average of the two.
	 */
	public static final int kEncoderUnitsPerRotation = 51711;
	/**
	 * Using the configSelectedFeedbackCoefficient() function, scale units to 3600 per rotation.
	 * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
	 */
	public static final double kTurnTravelUnitsPerRotation = 3600.0;
    public static final double kFeedbackCoefficient = kTurnTravelUnitsPerRotation / kEncoderUnitsPerRotation;
    
    private final BaseTalon _controller;
    private final int _controllerType;
    private final Convertor _convertor;
    private final Faults _faults = new Faults();
    private final SimpleMotorFeedforward _feedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);

    private FeedbackDevice _feedbackDevice = FeedbackDevice.None;
    private double _setpoint = 0.0;          // The most recently set setpoint.
    private double _nativeSetpoint = 0.0;    // The setpoint in native units. Field to avoid garbage collection.

    /** Cached values for various sensor readings. */
    private double _cachedVelocity = Double.NaN;
    private double _cachedPosition = Double.NaN;


    public SmartMotorHelper( final BaseTalon talon, final InvertType invert ) {
        _controller = talon;
        _controllerType = _controller.getClass().getSimpleName() == "WPI_TalonFX" ? kTalonFX : kTalonSRX;

        _feedbackDevice = kDefaultFeedbackDevice[_controllerType];
        _convertor = new Convertor(kSensorUnitsPerRotation[_controllerType]);
        _convertor.setRatios(kDefaultGearRatio);
        
        _controller.configFactoryDefault();
        _controller.setInverted(invert);
    }

    public void initController() {

        /* Configure Sensor Source for Primary PID */
        _controller.configSelectedFeedbackSensor(_feedbackDevice, PID_PRIMARY, kTimeoutMs);

        /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
        _controller.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

        //_controller.setSensorPhase(false);
        //_controller.setInverted(invert);
        _controller.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        _controller.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
        _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
 
        /* Set the peak and nominal outputs */
		_controller.configNominalOutputForward(0, kTimeoutMs);
		_controller.configNominalOutputReverse(0, kTimeoutMs);
		_controller.configPeakOutputForward(1, kTimeoutMs);
        _controller.configPeakOutputReverse(-1, kTimeoutMs);
        
		/* Set Motion Magic gains in slot0 - see documentation */
        setClosedLoopGains(kSlot_Distanc, kDefaultGains_Distanc); 
        _controller.configMotionAcceleration(6000, kTimeoutMs);
        _controller.configMotionCruiseVelocity(15000, kTimeoutMs);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
         _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
         _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);

         _controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    }

    public void configureRatios( GearRatios gearRatio ) {
        _convertor.setRatios(gearRatio);
    }

    public void setClosedLoopGains(int slot, Gains gain ) {
		_controller.config_kP(slot, gain.kP, kTimeoutMs);
		_controller.config_kI(slot, gain.kI, kTimeoutMs);
		_controller.config_kD(slot, gain.kD, kTimeoutMs);
		_controller.config_kF(slot, gain.kF, kTimeoutMs);
		_controller.config_IntegralZone(slot, gain.kIzone, kTimeoutMs);
		_controller.configClosedLoopPeakOutput(slot, gain.kPeakOutput);
		_controller.configAllowableClosedloopError(slot, 0, kTimeoutMs);
//		_controller.configMaxIntegralAccumulator(slot, gain. maxIntegral, kTimeoutMs);

//      Set acceleration and vcruise velocity - see documentation
        _controller.configClosedLoopPeriod(slot, 1);
    }
	
    public void setDistanceConfigs(Gains gains) {
        setClosedLoopGains(kSlot_Distanc, gains);

        _controller.configMotionAcceleration(6000, kTimeoutMs);
        _controller.configMotionCruiseVelocity(15000, kTimeoutMs);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
        _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);

        _controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    }

    public void setDistanceConfigs(Gains gains, BaseTalon auxTalon ) {    
        _controller.configRemoteFeedbackFilter(auxTalon.getDeviceID(), 
                                                RemoteSensorSource.TalonSRX_SelectedSensor, 
                                                REMOTE_0);
        /* Check if we're inverted */
        if (_controller.getInverted()) {
            _controller.configSensorTerm(SensorTerm.Diff0, _feedbackDevice);
            _controller.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0);
            _controller.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, PID_PRIMARY, kTimeoutMs);
        } else {
            /* Master is not inverted, both sides are positive so we can sum them. */
            _controller.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
            _controller.configSensorTerm(SensorTerm.Sum1, _feedbackDevice);
            _controller.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_PRIMARY, kTimeoutMs);
        }
        _controller.configSelectedFeedbackCoefficient(0.5, PID_PRIMARY, kTimeoutMs);
    }
    
    public void setDistanceAndTurnConfigs(Gains dgains, Gains gains, BaseTalon auxTalon) {
        setDistanceConfigs(dgains, auxTalon);

		/* Check if we're inverted */
		if (_controller.getInverted()) {
            _controller.configSensorTerm(SensorTerm.Sum0, _feedbackDevice);
            _controller.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor0);
            _controller.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_TURN, kTimeoutMs);
            _controller.configAuxPIDPolarity(true, kTimeoutMs);
        } else {
            /* Master is not inverted, both sides are positive so we can diff them. */
            _controller.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0);
            _controller.configSensorTerm(SensorTerm.Diff1, _feedbackDevice);
            _controller.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, PID_TURN, kTimeoutMs);
            _controller.configAuxPIDPolarity(true, kTimeoutMs);
		}
        _controller.configSelectedFeedbackCoefficient(kFeedbackCoefficient, PID_TURN, kTimeoutMs);

        this.setClosedLoopGains(kSlot_Turning, gains);

		_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, kTimeoutMs);
		_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10, kTimeoutMs);
		_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 10, kTimeoutMs);
		_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
        _controller.selectProfileSlot(kSlot_Turning, PID_TURN);

        auxTalon.follow(_controller, FollowerType.AuxOutput1);
    }

    /**
     * Get the velocity of the drive.
     *
     * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
     */
    public Double getVelocity(BaseTalon talon) {
        return _convertor.nativeUnitsToVelocity(_controller.getSelectedSensorVelocity(PID_PRIMARY));
    }
    /**
     * Get the velocity of the drive.
     *
     * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
     */
    public Double getAuxVelocity(BaseTalon talon) {
        return _convertor.nativeUnitsToVelocity(_controller.getSelectedSensorVelocity(PID_TURN));
    }

    /**
     * Get the position of the drive.
     *
     * @return The signed position in meters, or null if the drive doesn't have encoders.
     */
    public Double getPosition(BaseTalon talon) {
        return _convertor.nativeUnitsToDistanceMeters(_controller.getSelectedSensorPosition(PID_PRIMARY));
    }

    /**
     * Get the position2 of the drive.
     *
     * @return The signed position in meters, or null if the drive doesn't have encoders.
     */
    public Double getAuxPosition(BaseTalon talon) {
        return _convertor.nativeUnitsToDistanceMeters(_controller.getSelectedSensorPosition(PID_TURN));
    }

    /**
     * Get the cached velocity of the drive.
     *
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    public Double getVelocityCached() {
        return _cachedVelocity;
    }

    /**
     * Get the cached position of the drive.
     *
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    public Double getPositionCached() {
        return _cachedPosition;
    }

    public void setTarget(double meters) {
        _controller.set(ControlMode.MotionMagic, _convertor.distanceMetersToNativeUnits(meters));
    }
    public void setTarget(double meters, double aux) {
        _controller.set(ControlMode.MotionMagic, _convertor.distanceMetersToNativeUnits(meters), DemandType.AuxPID, aux);
    }

    public void setVelocityUPS(final double velocity) {
        _nativeSetpoint = _convertor.velocityToNativeUnits(velocity);
        _setpoint = velocity;

        _controller.config_kF(0, 0, 0);
        _controller.set(
            ControlMode.Velocity,
            _nativeSetpoint,
            DemandType.ArbitraryFeedForward,
            _feedForwardCalculator.calculate(velocity) / 12.);
    }

    public double getSetpoint() {
        return _setpoint;
    }
    public void setSetpoint(double pt) {
        _setpoint = pt;
    }
    /** Resets the position of the Talon to 0. */
    public void resetPosition(BaseTalon talon) {
        _controller.setSelectedSensorPosition(0, PID_PRIMARY, kTimeoutMs);
        // _controller.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
    }
    public void enableBrakes(boolean enabled) {
        _controller.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /** Updates all cached values with current ones. */
    public void update(BaseTalon talon) {
        _cachedVelocity = getVelocity(talon);
        _cachedPosition = getPosition(talon);

        _controller.getFaults(_faults);
        if (_faults.SensorOutOfPhase) {
            double leftVelUnitsPer100ms = _controller.getSelectedSensorVelocity(0);
            System.out.println("sensor is out of phase: " + leftVelUnitsPer100ms);
        }
    }
}
