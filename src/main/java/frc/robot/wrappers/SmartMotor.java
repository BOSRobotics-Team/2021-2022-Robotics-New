package frc.robot.wrappers;

import javax.annotation.Nullable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Convertor;
import frc.robot.Gains;

public class SmartMotor extends WPI_TalonFX {

    private final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.

    final private Convertor convertor;

    private SimpleMotorFeedforward feedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);
    private final Faults faults = new Faults();

    private String name = this.getDescription();
    /** Cached values for various sensor readings. */
    private double cachedVelocity = Double.NaN;
    private double cachedPosition = Double.NaN;

    /** The most recently set setpoint. */
    private double setpoint;

    /** The setpoint in native units. Field to avoid garbage collection. */
    private double nativeSetpoint;

    public SmartMotor( int deviceNumber, TalonFXInvertType invert ) {
        super(deviceNumber);
        convertor = new Convertor(kCountsPerRev);

        this.configFactoryDefault();

		/* Configure Sensor Source for Primary PID */
        this.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
        this.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

 		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
         */
        //this.setSensorPhase(false);
        this.setInverted(invert);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
        this.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        this.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, Constants.kTimeoutMs);
        
        /* Set the peak and nominal outputs */
		this.configNominalOutputForward(0, Constants.kTimeoutMs);
		this.configNominalOutputReverse(0, Constants.kTimeoutMs);
		this.configPeakOutputForward(1, Constants.kTimeoutMs);
        this.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        
		/* Set Motion Magic gains in slot0 - see documentation */
		this.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        this.setClosedLoopGains(Constants.kSlotIdx, Constants.kGains_Distanc); 

   		/* Set acceleration and vcruise velocity - see documentation */
        this.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
        this.configMotionAcceleration(6000, Constants.kTimeoutMs);
    }
   
    public void setClosedLoopGains(int slot, Gains gain ) {
		this.config_kP(slot, gain.kP, Constants.kTimeoutMs);
		this.config_kI(slot, gain.kI, Constants.kTimeoutMs);
		this.config_kD(slot, gain.kD, Constants.kTimeoutMs);
		this.config_kF(slot, gain.kF, Constants.kTimeoutMs);
		this.config_IntegralZone(slot, gain.kIzone, Constants.kTimeoutMs);
		this.configClosedLoopPeakOutput(slot, gain.kPeakOutput);
//		this.configMaxIntegralAccumulator(slot, gain. maxIntegral, Constants.kTimeoutMs);
    }
	
    public void setDistanceConfigs(TalonFXConfiguration talonConfig, Gains gains) {
		/* Configure the left Talon's selected sensor as local Integrated Sensor */
		talonConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();	// Local Feedback Source
		talonConfig.neutralDeadband = Constants.kNeutralDeadband;

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
	 }
	 public void setTurnConfigs(TalonFXConfiguration talonConfig, Gains gains) {
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
			talonConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
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
		talonConfig.auxiliaryPID.selectedFeedbackCoefficient = Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation;
	 }



    /**
     * Get the velocity of the drive.
     *
     * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getVelocity() {
        return convertor.nativeUnitsToVelocity(this.getSelectedSensorVelocity(0));
    }

    /**
     * Get the position of the drive.
     *
     * @return The signed position in meters, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getPosition() {
        return convertor.nativeUnitsToDistanceMeters(this.getSelectedSensorPosition(0));
    }

    /**
     * Get the cached velocity of the drive.
     *
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getVelocityCached() {
        return cachedVelocity;
    }

    /**
     * Get the cached position of the drive.
     *
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    @Nullable
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
    // Warn the user if they're setting Vbus to a number that's outside the range of values.
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
    public double getOutputCurrent() {
        return this.getSupplyCurrent();
    }
    /** Resets the position of the Talon to 0. */
    public void resetPosition() {
        this.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
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
        SmartDashboard.putNumber(name + " SensorVel", this.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
        SmartDashboard.putNumber(name + " SensorPos", this.getSelectedSensorPosition(Constants.kPIDLoopIdx));
        SmartDashboard.putNumber(name + " MotorOutputPercent", this.getMotorOutputPercent());
//        SmartDashboard.putNumber(name + "ClosedLoopError", this.getClosedLoopError(Constants.kPIDLoopIdx));
        SmartDashboard.putString(name + "ControlMode", this.getControlMode().toString());
        
        /* Print the Active Trajectory Point Motion Magic is servoing towards */
//        SmartDashboard.putNumber(name + "ClosedLoopTarget", this.getClosedLoopTarget(Constants.kPIDLoopIdx));
//        SmartDashboard.putNumber(name + "ActTrajVelocity", this.getActiveTrajectoryVelocity());
//        SmartDashboard.putNumber(name + "ActTrajPosition", this.getActiveTrajectoryPosition());        
    }

    public void enable() {
        this.set(ControlMode.PercentOutput, 0);
    }
}
