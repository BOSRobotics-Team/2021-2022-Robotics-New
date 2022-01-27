package frc.robot.wrappers;

import frc.robot.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartMotorController<T> implements MotorController {
	/** ---- Flat constants, you should not need to change these ---- */
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
    public static final int kSensorUnitsPerRotationFX = 2048;
    public static final int kSensorUnitsPerRotationSRX = 4096;

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
    
    public final T controller;
    private final Convertor convertor;
    private final Faults faults = new Faults();

    private FeedbackDevice feedbackDevice = FeedbackDevice.None;
    private SimpleMotorFeedforward feedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);
    private String name = "SmartMotorController";

    /** The most recently set setpoint. */
    private double setpoint = 0.0;

    /** The setpoint in native units. Field to avoid garbage collection. */
    private double nativeSetpoint = 0.0;

    /** Cached values for various sensor readings. */
    private double cachedVelocity = Double.NaN;
    private double cachedPosition = Double.NaN;


    public SmartMotorController( T ctrller ) {
        controller = ctrller;

        getBaseTalon().configFactoryDefault();

        if (controller.getClass().getSimpleName() == "WPI_TalonFX") {
            convertor = new Convertor(kSensorUnitsPerRotationFX);
            name = getTalonFX().getDescription();
            feedbackDevice = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        } else {
            convertor = new Convertor(kSensorUnitsPerRotationSRX);
            name = getTalonSRX().getDescription();
            feedbackDevice = FeedbackDevice.CTRE_MagEncoder_Relative;
        }
        convertor.setRatios(kDefaultGearRatio);

        /* Configure Sensor Source for Primary PID */
        getBaseTalon().configSelectedFeedbackSensor(feedbackDevice, PID_PRIMARY, kTimeoutMs);

        /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
        getBaseTalon().configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

        //getBaseTalon().setSensorPhase(false);
        //getBaseTalon().setInverted(invert);
        getBaseTalon().configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        getBaseTalon().configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
        getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
 
        /* Set the peak and nominal outputs */
		getBaseTalon().configNominalOutputForward(0, kTimeoutMs);
		getBaseTalon().configNominalOutputReverse(0, kTimeoutMs);
		getBaseTalon().configPeakOutputForward(1, kTimeoutMs);
        getBaseTalon().configPeakOutputReverse(-1, kTimeoutMs);
        
		/* Set Motion Magic gains in slot0 - see documentation */
        setClosedLoopGains(kSlot_Distanc, kDefaultGains_Distanc); 
        getBaseTalon().configMotionAcceleration(6000, kTimeoutMs);
        getBaseTalon().configMotionCruiseVelocity(15000, kTimeoutMs);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
         getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
         getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);

         getBaseTalon().selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    }
   public WPI_TalonFX getTalonFX() { return (WPI_TalonFX)controller; }
   public WPI_TalonSRX getTalonSRX() { return (WPI_TalonSRX)controller; }
   public BaseTalon getBaseTalon() { return (BaseTalon)controller; }

    public void configureRatios( GearRatios gearRatio ) {
        convertor.setRatios(gearRatio);
    }

    public void setClosedLoopGains(int slot, Gains gain ) {
		getBaseTalon().config_kP(slot, gain.kP, kTimeoutMs);
		getBaseTalon().config_kI(slot, gain.kI, kTimeoutMs);
		getBaseTalon().config_kD(slot, gain.kD, kTimeoutMs);
		getBaseTalon().config_kF(slot, gain.kF, kTimeoutMs);
		getBaseTalon().config_IntegralZone(slot, gain.kIzone, kTimeoutMs);
		getBaseTalon().configClosedLoopPeakOutput(slot, gain.kPeakOutput);
		getBaseTalon().configAllowableClosedloopError(slot, 0, kTimeoutMs);
//		getBaseTalon().configMaxIntegralAccumulator(slot, gain. maxIntegral, kTimeoutMs);

//      Set acceleration and vcruise velocity - see documentation
        getBaseTalon().configClosedLoopPeriod(slot, 1);
    }
	
    public void setDistanceConfigs(Gains gains) {
        setClosedLoopGains(kSlot_Distanc, gains);

        getBaseTalon().configMotionAcceleration(6000, kTimeoutMs);
        getBaseTalon().configMotionCruiseVelocity(15000, kTimeoutMs);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
        getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);

        getBaseTalon().selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
    }

    public void setDistanceConfigs(Gains gains, SmartMotorController<T> auxController ) {    
            getBaseTalon().configRemoteFeedbackFilter(auxController.getBaseTalon().getDeviceID(), 
                                                      RemoteSensorSource.TalonSRX_SelectedSensor, 
                                                      REMOTE_0);

            /* Check if we're inverted */
            if (this.getInverted()) {
                getBaseTalon().configSensorTerm(SensorTerm.Diff0, feedbackDevice);
                getBaseTalon().configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0);
                getBaseTalon().configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, PID_PRIMARY, kTimeoutMs);
            } else {
                /* Master is not inverted, both sides are positive so we can sum them. */
                getBaseTalon().configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
                getBaseTalon().configSensorTerm(SensorTerm.Sum1, feedbackDevice);
                getBaseTalon().configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_PRIMARY, kTimeoutMs);
            }
            getBaseTalon().configSelectedFeedbackCoefficient(0.5, PID_PRIMARY, kTimeoutMs);
    }
    
    public void setDistanceAndTurnConfigs(Gains dgains, Gains gains, SmartMotorController<T> auxController) {
        setDistanceConfigs(dgains, auxController);

		/* Check if we're inverted */
		if (this.getInverted()) {
            getTalonFX().configSensorTerm(SensorTerm.Sum0, feedbackDevice);
            getTalonFX().configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor0);
            getBaseTalon().configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_TURN, kTimeoutMs);
            getBaseTalon().configAuxPIDPolarity(true, kTimeoutMs);
        } else {
            /* Master is not inverted, both sides are positive so we can diff them. */
            getTalonSRX().configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor1);
            getTalonSRX().configSensorTerm(SensorTerm.Diff1, feedbackDevice);
            getBaseTalon().configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, PID_TURN, kTimeoutMs);
            getBaseTalon().configAuxPIDPolarity(true, kTimeoutMs);
		}
        getBaseTalon().configSelectedFeedbackCoefficient(kFeedbackCoefficient, PID_TURN, kTimeoutMs);

        this.setClosedLoopGains(kSlot_Turning, gains);

		getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, kTimeoutMs);
		getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10, kTimeoutMs);
		getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 10, kTimeoutMs);
		getBaseTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
        getBaseTalon().selectProfileSlot(kSlot_Turning, PID_TURN);

        auxController.getBaseTalon().follow(getBaseTalon(), FollowerType.AuxOutput1);
    }

    /**
     * Common interface for setting the speed of a motor controller.
     *
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void set(double speed) {
        getTalonSRX().set(speed);
    }

    /**
     * Common interface for getting the current set speed of a motor controller.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double get() { 
        return getTalonSRX().get(); 
    }
   /**
     * Set the rotation inversion of the motor.
     *
     */
    public void setInverted(boolean isInverted) {
        getBaseTalon().setInverted(isInverted);
    }
    public void setInverted(InvertType invertType) {
        getBaseTalon().setInverted(invertType);
    }
    /**
     * Return if a motor controller is in the inverted state or not.
     *
     * @return isInverted The state of the inversion true is inverted.
     */
    public boolean getInverted() { return getBaseTalon().getInverted(); }

    /** Disable the motor controller. */
    public void disable() { getBaseTalon().set(ControlMode.Disabled, 0); }

    /**
     * Stops motor movement. Motor can be moved again by calling set without having to re-enable the
     * motor.
     */
    public void stopMotor()  { getBaseTalon().set(ControlMode.Disabled, 0); }

    /**
     * Get the velocity of the drive.
     *
     * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
     */
    public Double getVelocity() {
        return convertor.nativeUnitsToVelocity(getBaseTalon().getSelectedSensorVelocity(0));
    }

    /**
     * Get the position of the drive.
     *
     * @return The signed position in meters, or null if the drive doesn't have encoders.
     */
    public Double getPosition() {
        return convertor.nativeUnitsToDistanceMeters(getBaseTalon().getSelectedSensorPosition(0));
    }

    /**
     * Get the cached velocity of the drive.
     *
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    public Double getVelocityCached() {
        return cachedVelocity;
    }

    /**
     * Get the cached position of the drive.
     *
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    public Double getPositionCached() {
        return cachedPosition;
    }

    public void setTarget(double meters) {
        getBaseTalon().set(ControlMode.MotionMagic, convertor.distanceMetersToNativeUnits(meters));
    }
    public void setTarget(double meters, double aux) {
        getBaseTalon().set(ControlMode.MotionMagic, convertor.distanceMetersToNativeUnits(meters), DemandType.AuxPID, aux);
    }

    public void setPercentVoltage(double pctVolts) {
        if (Math.abs(pctVolts) > 1.0) {
            Shuffleboard.addEventMarker(
                "WARNING: YOU ARE CLIPPING MAX PERCENT VBUS AT L:" + pctVolts,
                getClass().getSimpleName(),
                EventImportance.kNormal);

                pctVolts = Math.signum(pctVolts);
        }
        setpoint = pctVolts;

        getBaseTalon().set(ControlMode.PercentOutput, pctVolts);
    }
    
    public void setVelocityUPS(final double velocity) {
        nativeSetpoint = convertor.velocityToNativeUnits(velocity);
        setpoint = velocity;

        getBaseTalon().config_kF(0, 0, 0);
        getBaseTalon().set(
            ControlMode.Velocity,
            nativeSetpoint,
            DemandType.ArbitraryFeedForward,
            feedForwardCalculator.calculate(velocity) / 12.);
    }

    public double getSetpoint() {
        return setpoint;
    }
    public void setSetpoint(double pt) {
        setpoint = pt;
    }
    public double getOutputVoltage() {
        return getBaseTalon().getMotorOutputVoltage();
    }
    public double getBatteryVoltage() {
        return getBaseTalon().getBusVoltage();
    }
    /** Resets the position of the Talon to 0. */
    public void resetPosition() {
        getBaseTalon().setSelectedSensorPosition(0, PID_PRIMARY, kTimeoutMs);
        // getBaseTalon().getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
    }
    public void enableBrakes(boolean enabled) {
        getBaseTalon().setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public String getName() {
        return name;
    }
    public void setName(String nm) {
        name = nm;
    }

    /** Updates all cached values with current ones. */
    public void update() {
        cachedVelocity = getVelocity();
        cachedPosition = getPosition();

        getBaseTalon().getFaults(faults);
        if (faults.SensorOutOfPhase) {
            double leftVelUnitsPer100ms = getBaseTalon().getSelectedSensorVelocity(0);
            System.out.println("sensor is out of phase: " + leftVelUnitsPer100ms);
        }
    }

    public void logPeriodic() {
        String name = this.getName() + " ";

        SmartDashboard.putNumber(name + " cachePos: ", cachedPosition);
        SmartDashboard.putNumber(name + " cacheVel: ", cachedVelocity);

        /* Smart dash plots */
        SmartDashboard.putNumber(name + " SensorVel", getBaseTalon().getSelectedSensorVelocity(PID_PRIMARY));
        SmartDashboard.putNumber(name + " SensorPos", getBaseTalon().getSelectedSensorPosition(PID_PRIMARY));
        SmartDashboard.putNumber(name + " MotorOutputPercent", getBaseTalon().getMotorOutputPercent());
        SmartDashboard.putString(name + "ControlMode", getBaseTalon().getControlMode().toString());
//      SmartDashboard.putNumber(name + "ClosedLoopError", getBaseTalon().getClosedLoopError(PID_PRIMARY));
        
        /* Print the Active Trajectory Point Motion Magic is servoing towards */
//        SmartDashboard.putNumber(name + "ClosedLoopTarget", getBaseTalon().getClosedLoopTarget(PID_PRIMARY));
//        SmartDashboard.putNumber(name + "ActTrajVelocity", getBaseTalon().getActiveTrajectoryVelocity());
//        SmartDashboard.putNumber(name + "ActTrajPosition", getBaseTalon().getActiveTrajectoryPosition());        
    }

    public void enable() {
        getBaseTalon().set(ControlMode.PercentOutput, 0);
    }
}
