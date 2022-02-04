package frc.robot.wrappers;

import frc.robot.*;
import frc.robot.util.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartMotor extends WPI_TalonFX {

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
    public static final int kSensorUnitsPerRotation = 2048;
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
    
    final private Convertor convertor = new Convertor(kSensorUnitsPerRotation);

    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private SimpleMotorFeedforward feedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);
    private final Faults faults = new Faults();

    private String name = this.getDescription();
    /** Cached values for various sensor readings. */
    private double cachedVelocity = Double.NaN;
    private double cachedPosition = Double.NaN;

    /** The most recently set setpoint. */
    private double setpoint = 0.0;

    /** The setpoint in native units. Field to avoid garbage collection. */
    private double nativeSetpoint = 0.0;

    public SmartMotor( int deviceNumber ) {
        super(deviceNumber);

        this.configFactoryDefault();

		/* Configure Sensor Source for Primary PID */
        this.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_PRIMARY, kTimeoutMs);

        /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
        this.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

 		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
         */
        //this.setSensorPhase(false);
        //this.setInverted(invert);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
        this.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        this.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
        
        /* Set the peak and nominal outputs */
		this.configNominalOutputForward(0, kTimeoutMs);
		this.configNominalOutputReverse(0, kTimeoutMs);
		this.configPeakOutputForward(1, kTimeoutMs);
        this.configPeakOutputReverse(-1, kTimeoutMs);
        
		/* Set Motion Magic gains in slot0 - see documentation */
		this.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);
        this.setClosedLoopGains(kSlot_Distanc, kDefaultGains_Distanc); 

   		/* Set acceleration and vcruise velocity - see documentation */
        this.configMotionCruiseVelocity(15000, kTimeoutMs);
        this.configMotionAcceleration(6000, kTimeoutMs);

        convertor.setRatios(kDefaultGearRatio);
    }
   
    public void configureRatios( GearRatios gearRatio ) {
        convertor.setRatios(gearRatio);
    }

    public void setClosedLoopGains(int slot, Gains gain ) {
		this.config_kP(slot, gain.kP, kTimeoutMs);
		this.config_kI(slot, gain.kI, kTimeoutMs);
		this.config_kD(slot, gain.kD, kTimeoutMs);
		this.config_kF(slot, gain.kF, kTimeoutMs);
		this.config_IntegralZone(slot, gain.kIzone, kTimeoutMs);
		this.configClosedLoopPeakOutput(slot, gain.kPeakOutput);
//		this.configMaxIntegralAccumulator(slot, gain. maxIntegral, kTimeoutMs);
    }
	
    public void setDistanceConfigs(Gains gains) {
        this.getAllConfigs(talonConfig);
        System.out.println(this.getName() + " - Config(before): " + talonConfig);

        /* Configure the left Talon's selected sensor as local Integrated Sensor */
		talonConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();	// Local Feedback Source
		talonConfig.neutralDeadband = kNeutralDeadband;

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		talonConfig.nominalOutputForward = 0.0;
		talonConfig.nominalOutputReverse = 0.0;
		talonConfig.peakOutputForward = +1.0;
		talonConfig.peakOutputReverse = -1.0;

		/* FPID Gains for distance servo */
		talonConfig.slot0.kP = gains.kP;
		talonConfig.slot0.kI = gains.kI;
		talonConfig.slot0.kD = gains.kD;
		talonConfig.slot0.kF = gains.kF;
		talonConfig.slot0.integralZone = gains.kIzone;
		talonConfig.slot0.closedLoopPeakOutput = gains.kPeakOutput;
		talonConfig.slot0.allowableClosedloopError = 0;

		int closedLoopTimeMs = 1;
		talonConfig.slot0.closedLoopPeriod = closedLoopTimeMs;

		/* Motion Magic Configurations */
		talonConfig.motionAcceleration = 6000;
		talonConfig.motionCruiseVelocity = 15000;

		/* Check if we're inverted */
		if (this.getInverted()) {
			talonConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			talonConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			//masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Diff0 - Diff1
		} else {
			/* Master is not inverted, both sides are positive so we can sum them. */
			talonConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();    //Aux Selected Sensor
			talonConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			//masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
		}
		/* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double  the real-world value */
		//masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;

        this.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        this.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
		this.selectProfileSlot(kSlot_Distanc, PID_PRIMARY);

        System.out.println(this.getName() + " - Config(to set): " + talonConfig);
		this.configAllSettings(talonConfig);

    }
	
    public void setDistanceAndTurnConfigs(int sensorID, Gains dgains, Gains gains) {
        this.setDistanceConfigs(dgains);

        /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		talonConfig.remoteFilter0.remoteSensorDeviceID = sensorID; // Device ID of Source
		talonConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; // Remote Feedback Source

        /* FPID Gains for distance servo */
		talonConfig.slot1.kP = gains.kP;
		talonConfig.slot1.kI = gains.kI;
		talonConfig.slot1.kD = gains.kD;
		talonConfig.slot1.kF = gains.kF;
		talonConfig.slot1.integralZone = gains.kIzone;
		talonConfig.slot1.closedLoopPeakOutput = gains.kPeakOutput;
		talonConfig.slot1.allowableClosedloopError = 0;

		int closedLoopTimeMs = 1;
		talonConfig.slot1.closedLoopPeriod = closedLoopTimeMs;

		/* Check if we're inverted */
		if (this.getInverted()) {
			talonConfig.sum0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			talonConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();   //Aux Selected Sensor
			talonConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
			talonConfig.auxPIDPolarity = true;
		} else {
			/* Master is not inverted, both sides are positive so we can diff them. */
			talonConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
			talonConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			talonConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
			/* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
			talonConfig.auxPIDPolarity = true;
		}
		talonConfig.auxiliaryPID.selectedFeedbackCoefficient = kFeedbackCoefficient;

		setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, kTimeoutMs);
		setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10, kTimeoutMs);
		setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 10, kTimeoutMs);
		setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
        selectProfileSlot(kSlot_Turning, PID_TURN);

        System.out.println(this.getName() + " - TurnConfig(to set): " + talonConfig);
		this.configAllSettings(talonConfig);
    }

    /**
     * Set the rotation inversion of the motor.
     *
     */
    public void setInverted(InvertType invert) {
        switch (invert) {
            case FollowMaster:
                break;
            case InvertMotorOutput:
                this.setInverted(TalonFXInvertType.CounterClockwise);
                break;
            case None:
                this.setInverted(TalonFXInvertType.Clockwise);
                break;
            case OpposeMaster:
                break;
            default:
                break;
        }
    }

    /**
     * Get the velocity of the drive.
     *
     * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
     */
    public Double getVelocity() {
        return convertor.nativeUnitsToVelocity(this.getSelectedSensorVelocity(0));
    }

    /**
     * Get the position of the drive.
     *
     * @return The signed position in meters, or null if the drive doesn't have encoders.
     */
    public Double getPosition() {
        return convertor.nativeUnitsToDistanceMeters(this.getSelectedSensorPosition(0));
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
        this.set(TalonFXControlMode.MotionMagic, convertor.distanceMetersToNativeUnits(meters));
    }
    public void setTarget(double meters, double aux) {
        this.set(TalonFXControlMode.MotionMagic, convertor.distanceMetersToNativeUnits(meters), DemandType.AuxPID, aux);
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

        this.set(ControlMode.PercentOutput, pctVolts);
    }
    
    public void setVelocityUPS(final double velocity) {
        nativeSetpoint = convertor.velocityToNativeUnits(velocity);
        setpoint = velocity;

        this.config_kF(0, 0, 0);
        this.set(
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
        return this.getMotorOutputVoltage();
    }
    public double getBatteryVoltage() {
        return this.getBusVoltage();
    }
    /** Resets the position of the Talon to 0. */
    public void resetPosition() {
        this.setSelectedSensorPosition(0, PID_PRIMARY, kTimeoutMs);
        this.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
    }
    public void enableBrakes(boolean enabled) {
        this.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
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

        this.getFaults(faults);
        if (faults.SensorOutOfPhase) {
            double leftVelUnitsPer100ms = this.getSelectedSensorVelocity(0);
            System.out.println("sensor is out of phase: " + leftVelUnitsPer100ms);
        }
    }

    public void logPeriodic() {
        String name = this.getName() + " ";

        SmartDashboard.putNumber(name + " cachePos: ", cachedPosition);
        SmartDashboard.putNumber(name + " cacheVel: ", cachedVelocity);

        /* Smart dash plots */
        SmartDashboard.putNumber(name + " SensorVel", this.getSelectedSensorVelocity(PID_PRIMARY));
        SmartDashboard.putNumber(name + " SensorPos", this.getSelectedSensorPosition(PID_PRIMARY));
        SmartDashboard.putNumber(name + " MotorOutputPercent", this.getMotorOutputPercent());
        SmartDashboard.putString(name + "ControlMode", this.getControlMode().toString());
//      SmartDashboard.putNumber(name + "ClosedLoopError", this.getClosedLoopError(PID_PRIMARY));
        
        /* Print the Active Trajectory Point Motion Magic is servoing towards */
//        SmartDashboard.putNumber(name + "ClosedLoopTarget", this.getClosedLoopTarget(PID_PRIMARY));
//        SmartDashboard.putNumber(name + "ActTrajVelocity", this.getActiveTrajectoryVelocity());
//        SmartDashboard.putNumber(name + "ActTrajPosition", this.getActiveTrajectoryPosition());        
    }

    public void enable() {
        this.set(ControlMode.PercentOutput, 0);
    }
}
