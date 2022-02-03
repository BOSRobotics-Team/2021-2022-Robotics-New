package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SmartMotorHelper {
    public static final int kMain = 0;
    public static final int kAux = 1;
    
	/** ---- Flat constants, you should not need to change these ---- */
    public static final int kTalonNone = 0;
    public static final int kTalonSRX = 1;
    public static final int kTalonFX = 2;

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

    public static final int kSensorUnitsPerRotation[] = { 4096, 4096, 2048 };
    public static final FeedbackDevice kDefaultFeedbackDevice[] = { FeedbackDevice.None, FeedbackDevice.CTRE_MagEncoder_Relative, FeedbackDevice.IntegratedSensor };

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
    private final BaseTalon _auxController;

    private final int _controllerType;
    private final Convertor _convertor;
    private final Faults _faults = new Faults();
    private final SimpleMotorFeedforward _feedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);

    private FeedbackDevice _feedbackDevice = FeedbackDevice.None;

    private double _setpoint = 0.0;          // The most recently set setpoint.
    private double _nativeSetpoint = 0.0;    // The setpoint in native units. Field to avoid garbage collection.
    private double _auxpoint = 0.0;          // The most recently set setpoint.
    private double _nativeAuxpoint = 0.0;    // The setpoint in native units. Field to avoid garbage collection.
    private boolean _isVelocity = false;
    private boolean _isDistance = false;
    private boolean _isDistanceAux = false;

    /** Cached values for various sensor readings. */
    private double _cachedVelocity = Double.NaN;
    private double _cachedPosition = Double.NaN;


    public SmartMotorHelper( final BaseTalon talon, final BaseTalon auxTalon ) {
        _controller = talon;
        _auxController = auxTalon;

        if (_controller.getClass() == WPI_TalonFX.class)
            _controllerType = kTalonFX;
        else if (_controller.getClass() == WPI_TalonSRX.class)
            _controllerType = kTalonSRX;
        else 
            _controllerType = kTalonNone;
        
        _convertor = new Convertor(kSensorUnitsPerRotation[_controllerType]);
        _convertor.setRatios(kDefaultGearRatio);
        
        _controller.configFactoryDefault();
        _feedbackDevice = kDefaultFeedbackDevice[_controllerType];

        if (_auxController != null) {
            _auxController.configFactoryDefault();
        }
    }

    public SmartMotorHelper( final BaseTalon talon ) {
        this(talon, null);
    }

    public void initController() {
        _isDistance = _isDistanceAux = _isVelocity = false;
        initController(_controller);
        if (_auxController != null)
            initController(_auxController);
    }

    public void configureRatios( GearRatios gearRatio ) {
        _convertor.setRatios(gearRatio);
    }

    private void initController(BaseTalon talon) {

        /* Configure Sensor Source for Primary PID */
        talon.configSelectedFeedbackSensor(_feedbackDevice, PID_PRIMARY, kTimeoutMs);

        /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
        talon.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

        //_controller.setSensorPhase(false);
        //_controller.setInverted(invert);
        talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
         talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
         talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
 
        /* Set the peak and nominal outputs */
		talon.configNominalOutputForward(0, kTimeoutMs);
		talon.configNominalOutputReverse(0, kTimeoutMs);
		talon.configPeakOutputForward(1, kTimeoutMs);
        talon.configPeakOutputReverse(-1, kTimeoutMs);
        
		/* Set Motion Magic gains in slot0 - see documentation */
        setClosedLoopGains(talon, kSlot_Distanc, kDefaultGains_Distanc); 
        talon.configMotionAcceleration(6000, kTimeoutMs);
        talon.configMotionCruiseVelocity(15000, kTimeoutMs);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
         talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
         talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);

         talon.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    }

    private void setClosedLoopGains(BaseTalon talon, int slot, Gains gain ) {
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

     	/* Set relevant frame periods to be at least as fast as periodic rate */
        _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);

        _controller.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);

        if (_auxController != null) {
            _controller.configRemoteFeedbackFilter(_auxController.getDeviceID(), 
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
            _auxController.follow(_controller, FollowerType.PercentOutput);
        }
    }

    public void setDistanceAndTurnConfigs(Gains dgains, Gains tgains) {
        setDistanceConfigs(dgains);

        setClosedLoopGains(_controller, kSlot_Turning, tgains);

		_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, kTimeoutMs);
		_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10, kTimeoutMs);
		_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 10, kTimeoutMs);
		_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
        _controller.selectProfileSlot(kSlot_Turning, PID_TURN);

        if (_auxController != null) {
            _controller.configRemoteFeedbackFilter(_auxController.getDeviceID(), 
                                                   RemoteSensorSource.TalonSRX_SelectedSensor, 
                                                   REMOTE_0);
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
            _auxController.follow(_controller, FollowerType.AuxOutput1);
        }
    }

    /**
     * Get the velocity of the drive.
     *
     * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
     */
    public Double getVelocity() {
        return _convertor.nativeUnitsToVelocity(_controller.getSelectedSensorVelocity(PID_PRIMARY));
    }
    public Double getNativeVelocity() {
        return _controller.getSelectedSensorVelocity(PID_PRIMARY);
    }
    /**
     * Get the velocity of the drive.
     *
     * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
     */
    public Double getAuxVelocity() {
        return _convertor.nativeUnitsToVelocity(_controller.getSelectedSensorVelocity(PID_TURN));
    }
    public Double getAuxNativeVelocity() {
        return _controller.getSelectedSensorVelocity(PID_TURN);
    }

    /**
     * Get the position of the drive.
     *
     * @return The signed position in meters, or null if the drive doesn't have encoders.
     */
    public Double getPosition() {
        return _convertor.nativeUnitsToDistanceMeters(_controller.getSelectedSensorPosition(PID_PRIMARY));
    }
    public Double getNativePosition() {
        return _controller.getSelectedSensorPosition(PID_PRIMARY);
    }

    /**
     * Get the position2 of the drive.
     *
     * @return The signed position in meters, or null if the drive doesn't have encoders.
     */
    public Double getAuxPosition() {
        return _convertor.nativeUnitsToDistanceMeters(_controller.getSelectedSensorPosition(PID_TURN));
    }
    public Double getAuxNativePosition() {
        return _controller.getSelectedSensorPosition(PID_TURN);
    }

    public void set(double pctOutput) {
        _isDistance = _isDistanceAux = _isVelocity = false;
        _controller.set(ControlMode.PercentOutput, pctOutput);
        if (_auxController != null)
            _auxController.follow(_controller, FollowerType.PercentOutput);
    }
    public void set(double pctOutput, double auxOutput) {
        _isDistance = _isDistanceAux = _isVelocity = false;
        _controller.set(ControlMode.PercentOutput, pctOutput);
        if (_auxController != null)
            _auxController.set(ControlMode.PercentOutput, auxOutput);
    }

    public void setTarget(double meters) {
        _isDistance = _isDistanceAux = _isVelocity = false;

        _setpoint = meters;
        _nativeSetpoint = _convertor.distanceMetersToNativeUnits(meters);

        if (_auxController != null)
            _auxController.follow(_controller, FollowerType.PercentOutput);
        _controller.set(ControlMode.MotionMagic, _nativeSetpoint);
        _isDistance = true;
    }
    public void setTarget(double meters, double aux) {
        _isDistance = _isDistanceAux = _isVelocity = false;

        _setpoint = meters;
        _nativeSetpoint = _convertor.distanceMetersToNativeUnits(meters);
        _auxpoint = aux;
        _nativeAuxpoint = aux;

        // System.out.println("target (units) = " + _convertor.distanceMetersToNativeUnits(meters) + " angle: " + aux);
        if (_auxController != null)
    		_auxController.follow(_controller, FollowerType.AuxOutput1);
        _controller.set(ControlMode.MotionMagic, _nativeSetpoint, DemandType.AuxPID, _nativeAuxpoint);
        _isDistanceAux = true;
    }

    public void setVelocityUPS(final double velocity) {
        _isDistance = _isDistanceAux = _isVelocity = false;

        _setpoint = velocity;
        _nativeSetpoint = _convertor.velocityToNativeUnits(_setpoint);
        _nativeAuxpoint = _feedForwardCalculator.calculate(velocity) / 12.;

        if (_auxController != null)
            _auxController.follow(_controller, FollowerType.PercentOutput);

        _controller.config_kF(0, 0, 0);
        _controller.set(
            ControlMode.Velocity,
            _nativeSetpoint,
            DemandType.ArbitraryFeedForward,
            _nativeAuxpoint);
        _isVelocity = true;
    }

    public boolean isFwdLimitSwitchClosed() { return _controller.isFwdLimitSwitchClosed() == 1; }
    public boolean isRevLimitSwitchClosed() { return _controller.isRevLimitSwitchClosed() == 1; }

    /** Resets the position of the Talon to 0. */
    public void resetPosition() {
        _controller.setSelectedSensorPosition(0, PID_PRIMARY, kTimeoutMs);
        if (_auxController != null) 
            _auxController.setSelectedSensorPosition(0, PID_PRIMARY, kTimeoutMs);
    }
    /** Resets the position of the Talon to 0. */
    public void resetAuxPosition() {
        _controller.setSelectedSensorPosition(0, PID_TURN, kTimeoutMs);
        if (_auxController != null) 
            _auxController.setSelectedSensorPosition(0, PID_TURN, kTimeoutMs);
    }
    public void enableBrakes(boolean enabled) {
        _controller.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
        if (_auxController != null) 
            _auxController.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public double getClosedLoopError() {
        return _controller.getClosedLoopError();
    }
    public double getActiveTrajectoryVelocity() {
        return _controller.getActiveTrajectoryVelocity();
    }

    /** Updates all cached values with current ones. */
    public void update() {
        if (_isDistance) {
            _controller.set(ControlMode.MotionMagic, _nativeSetpoint);
        } else if (_isDistanceAux) {
            _controller.set(ControlMode.MotionMagic, _nativeSetpoint, DemandType.AuxPID, _nativeAuxpoint);
        } else if (_isVelocity) {
            _controller.set(ControlMode.Velocity, _nativeSetpoint, DemandType.ArbitraryFeedForward, _nativeAuxpoint);    
        }

        // _cachedVelocity = getVelocity();
        // _cachedPosition = getPosition();

        _controller.getFaults(_faults);
        if (_faults.SensorOutOfPhase) {
            double leftVelUnitsPer100ms = _controller.getSelectedSensorVelocity(0);
            System.out.println("sensor is out of phase: " + leftVelUnitsPer100ms);
        }
    }
}
