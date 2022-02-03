// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;
import frc.robot.wrappers.*;
import frc.robot.sim.PhysicsSim;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

    public enum DriveMode {
        ARCADE,
        TANK,
        CURVATURE
    }

    private final WPI_TalonFX rightMaster = new WPI_TalonFX(0);
    private final WPI_TalonFX leftMaster = new WPI_TalonFX(1);
    private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(3);
    private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(2);

    // /* Object for simulated inputs into Talon. */
    // private final TalonFXSimCollection leftMasterSim = leftMaster.getSimCollection();
    // private final TalonFXSimCollection rightMasterSim = rightMaster.getSimCollection();
    
    public final SmartMotorHelper smartController = new SmartMotorHelper(leftMaster, rightMaster);

    /** Simulation stuff */
    // private final Convertor convertorSim = new Convertor(2048);
    // private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    //     DCMotor.getCIM(2),        //2 CIMS on each side of the drivetrain.
    //     Constants.kGearRatio,     //Standard AndyMark Gearing reduction.
    //     2.1,                      //MOI of 2.1 kg m^2 (from CAD model).
    //     26.5,                     //Mass of the robot is 26.5 kg.
    //     Units.inchesToMeters(Constants.kWheelRadiusInches),  //Robot uses 3" radius (6" diameter) wheels.
    //     0.546,                    //Distance between wheels is _ meters.
    //     null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
    // );

    /** The NavX gyro */
    private final DriveGyro gyro = new DriveGyro(false);

    /** Drivetrain kinematics processor for measuring individual wheel speeds */
    private final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Constants.kWidthChassisMeters);

    /** Drivetrain odometry tracker for tracking position */
    private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(gyro.getHeading());
    public final DifferentialDrive differentialDrive;

    private final Field2d m_field = new Field2d();

    // private boolean voltageCompEnabled = false;
    // private Double maxSpeed;

    private DriveMode m_DriveMode = DriveMode.ARCADE;
    private boolean m_UseSquares = true;
    private boolean m_UseDriveScaling = false;
    private double m_DriveScaling = 0.5;
    private boolean m_QuickTurn = false;

    private double _lastLSmoothing = 0.0;
    private double _lastRSmoothing = 0.0;

    public DriveTrain() {    
        leftMaster.setInverted(InvertType.None);
        rightMaster.setInverted(InvertType.InvertMotorOutput);
        
        smartController.initController();
        smartController.configureRatios(SmartMotorHelper.kDefaultGearRatio);
        smartController.enableBrakes(true);

        leftFollower.configFactoryDefault();
        leftFollower.follow(leftMaster);
        leftFollower.setInverted(InvertType.FollowMaster);

        rightFollower.configFactoryDefault();
        rightFollower.follow(rightMaster);
        rightFollower.setInverted(InvertType.FollowMaster);
  
        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
        differentialDrive.setSafetyEnabled(false);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(0.75);
        differentialDrive.setDeadband(0.02);

        // if (RobotBase.isSimulation()) {
        //     // convertorSim.setRatios(Constants.kGearRatio, Constants.kWheelRadiusInches, 1.0);
        //     PhysicsSim.getInstance().addTalonFX(leftMaster, 0.2, 6800);
		//     PhysicsSim.getInstance().addTalonFX(rightMaster, 0.2, 6800);
        // }

        resetPosition();
        addChild("Differential Drive", differentialDrive);
    }

    public void configForPID() {
        // resetPosition();
        smartController.setDistanceConfigs(Constants.kGains_Distanc);        
    }

    public void configForPID2() {
        // resetPosition();
        // setHeadingDegrees(0);
        smartController.setDistanceAndTurnConfigs(Constants.kGains_Distanc, Constants.kGains_Turning);        
    }

    public void configMotionSCurveStrength(int smoothing) {
        leftMaster.configMotionSCurveStrength(smoothing);
        rightMaster.configMotionSCurveStrength(smoothing);
        SmartDashboard.putNumber("Smoothing", smoothing);
    }

    public void setTarget(double distance) {
		smartController.setTarget(distance);
        differentialDrive.feed(); 
        // System.out.println("target (meters) = " + distance);
    }

    public void setTarget(double distance, double angle) {
		/* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
		smartController.setTarget(distance, angle);
        differentialDrive.feed(); 
		// System.out.println("target (meters) = " + distance + " angle: " + angle);
    }

    public Boolean isTargetReached(double target) {
        // double error = smartController.getClosedLoopError();
		// double velocity = smartController.getActiveTrajectoryVelocity();
		double position = smartController.getNativePosition();
        double targetPos = smartController.getPosition();

        System.out.println("AutonomousCommand - targetPos: " + targetPos + " pos: " + position);// + " vel: " + velocity);
        return (MathUtil.applyDeadband(targetPos - target, 0.1) == 0.0);
    }
    @Override
    public void periodic() {
        smartController.update();
    }

    @Override
    public void simulationPeriodic() {
        // // This method will be called once per scheduler run during simulation
        // /* Pass the robot battery voltage to the simulated Talon SRXs */
        // leftMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
        // rightMasterSim.setBusVoltage(RobotController.getBatteryVoltage());

        // driveSim.setInputs(leftMasterSim.getMotorOutputLeadVoltage(),
        //                    -rightMasterSim.getMotorOutputLeadVoltage());

        // /*
        // * Advance the model by 20 ms. Note that if you are running this
        // * subsystem in a separate thread or have changed the nominal
        // * timestep of TimedRobot, this value needs to match it.
        // */
        // driveSim.update(0.02);

        // // System.out.println("LeftMastSim  units: " + convertorSim.distanceMetersToNativeUnits(driveSim.getLeftPositionMeters()) + " meters: " + driveSim.getLeftPositionMeters());
        // // System.out.println("RightMastSim  units: " + convertorSim.distanceMetersToNativeUnits(driveSim.getRightPositionMeters()) + " meters: " + driveSim.getRightPositionMeters());

        // leftMasterSim.setIntegratedSensorRawPosition(convertorSim.distanceMetersToNativeUnits(driveSim.getLeftPositionMeters()));
        // leftMasterSim.setIntegratedSensorVelocity(convertorSim.velocityToNativeUnits(driveSim.getLeftVelocityMetersPerSecond()));
        // rightMasterSim.setIntegratedSensorRawPosition(convertorSim.distanceMetersToNativeUnits(-driveSim.getRightPositionMeters()));
        // rightMasterSim.setIntegratedSensorVelocity(convertorSim.velocityToNativeUnits(-driveSim.getRightVelocityMetersPerSecond()));
    }
    
    public Double getVelocity() { return smartController.getVelocity(); }
    public Double getPosition() { return smartController.getPosition(); }
    public Double getAuxPosition() { return smartController.getAuxPosition(); }

    /**
     * Get the velocity of the left side of the drive.
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    public Double getLeftVel() { return leftMaster.getSelectedSensorVelocity(0); }

    /**
     * Get the velocity of the right side of the drive.
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    public Double getRightVel() { return rightMaster.getSelectedSensorVelocity(0); }

    /**
     * Get the position of the left side of the drive.
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    public Double getLeftPos() { return leftMaster.getSelectedSensorPosition(0); }

    /**
     * Get the position of the right side of the drive.
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    public Double getRightPos() { return rightMaster.getSelectedSensorPosition(0); }

    /**
     * Get the position of the left side of the drive.
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    public Double getLeftAuxPos() { return leftMaster.getSelectedSensorVelocity(1); }

    /**
     * Get the position of the right side of the drive.
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    public Double getRightAuxPos() { return rightMaster.getSelectedSensorVelocity(1); }
    
    /** Completely stop the robot by setting the voltage to each side to be 0. */
    public void fullStop() {
        setPercentVoltage(0, 0);
        _lastLSmoothing = _lastRSmoothing = 0.0;
    }
  
    /** Reset odometry tracker to current robot pose */
    public void resetOdometry(final Pose2d pose) {
        resetPosition();
        setHeadingDegrees(pose.getRotation().getDegrees());
        driveOdometry.resetPosition(pose, getHeading());
//        driveOdometry.resetPosition(pose, ahrs.getRotation2d());
    }

    /** Update odometry tracker with current heading, and encoder readings */
    public void updateOdometry() {
        // need to convert to meters
//        double angle = ((getRightPos() - getLeftPos()) * (180.0 / Math.PI)) / Constants.kWidthChassisMeters;

        driveOdometry.update(/*Rotation2d.fromDegrees(angle), */getHeading(), 
                             getLeftPos(), 
                             getRightPos());
    
        m_field.setRobotPose(driveOdometry.getPoseMeters());
        SmartDashboard.putString("Heading", driveOdometry.getPoseMeters().getRotation().toString());
    }

    /** @return Current estimated pose based on odometry tracker data */
    public Pose2d getCurrentPose() {
        return driveOdometry.getPoseMeters() != null
            ? driveOdometry.getPoseMeters()
            : new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    }

    /** @return Current wheel speeds based on encoder readings for future pose correction */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVel(), getRightVel());
    }

    /** @return Kinematics processor for wheel speeds */
    public DifferentialDriveKinematics getDriveKinematics() {
        return driveKinematics;
    }

    /** Disable the motors. */
    public void disable() {
        leftMaster.disable();
        rightMaster.disable();
    }

    public void setPercentVoltage(double leftPctVolts, double rightPctVolts) {
        smartController.set(leftPctVolts, rightPctVolts);
    } 

    /** Resets the position of the Talon to 0. */
    public void resetPosition() {
        smartController.resetPosition();
        // leftMasterSim.setIntegratedSensorRawPosition(0);
        // rightMasterSim.setIntegratedSensorRawPosition(0);
    }
           
    public void logPeriodic() {
       // This method will be called once per scheduler run
       updateOdometry();

       /* Instrumentation */
//       Instrumentation.ProcessGyro(gyro);
//       Instrumentation.ProcessMotor(leftMaster);
//       Instrumentation.ProcessMotor(rightMaster);
       gyro.logPeriodic();

        SmartDashboard.putData("Field2d", m_field);
    }

    public void enableDriveTrain(boolean enable) {
        differentialDrive.setSafetyEnabled(enable);
        smartController.set(0.0, 0.0);
        if (!enable) {
            leftMaster.set(ControlMode.Disabled, 0.0);
            rightMaster.set(ControlMode.Disabled, 0.0);
        }
    }

    public void enableBrakes(boolean enabled) {
        smartController.enableBrakes(enabled);
    }

    public void zeroHeading() {
        gyro.zeroHeading();
    }
    public double getHeadingDegrees() {
        return gyro.getHeadingDegrees();
    }
    public Rotation2d getHeading() {
        return gyro.getHeading();
    }
    /**
     * Set the robot's heading.
     * @param heading The heading to set to, in degrees on [-180, 180].
     */
    public void setHeadingDegrees(final double heading) {
        gyro.setHeadingDegrees(heading);
    }

    public void setMaxOutput(double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
    }
    public void setRampRate(double rampTimeSeconds) {
        leftMaster.configOpenloopRamp(rampTimeSeconds);
        rightMaster.configOpenloopRamp(rampTimeSeconds);
    }
    
    // Put methods for controlling this subsystem here. Call these from Commands.
    public void driveArcade(double speed, double rotation, boolean useSquares) {
        differentialDrive.arcadeDrive(speed, rotation, useSquares);
    }
    public void driveTank(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }
    public void driveCurvature(double speed, double rotation, boolean quickTurn) {
        differentialDrive.curvatureDrive(speed, rotation, quickTurn);
    }
    public void driveToTarget(double meters) {
        smartController.setTarget(meters);
        differentialDrive.feed(); 
    }

    public void drive(XboxController ctrl) {
        if (m_DriveMode == DriveMode.ARCADE) {
            this.setOutput(ctrl.getLeftY(), -ctrl.getRightX());
        } else if (m_DriveMode == DriveMode.TANK) {
            this.setOutput(ctrl.getLeftY(), ctrl.getRightY());
        } else if (m_DriveMode == DriveMode.CURVATURE) {
            this.setOutput(ctrl.getLeftY(), -ctrl.getRightX());
        }
    }
    public void setOutput(double left, double right) {
        double newleft = (_lastLSmoothing + left) / 2.0;
        double newRight = (_lastRSmoothing + right) / 2.0;
        _lastLSmoothing = left;
        _lastRSmoothing = right;

        if (m_DriveMode == DriveMode.ARCADE) {
            this.driveArcade(newleft, newRight, m_UseSquares);
        } else if (m_DriveMode == DriveMode.TANK) {
            this.driveTank(newleft, newRight);
        } else if (m_DriveMode == DriveMode.CURVATURE) {
            this.driveCurvature(newleft, newRight, m_QuickTurn);
        }
    }
    public DriveMode getDriveMode() { return m_DriveMode; }
    public void setDriveMode(DriveMode mode) {
        m_DriveMode = mode;
        SmartDashboard.putString("DriveTrainMode", m_DriveMode.toString());
    }
    public boolean getUseSquares() { return m_UseSquares; }
    public void setUseSquares(boolean use) {
        m_UseSquares = use;
        SmartDashboard.putBoolean("UseSquares", m_UseSquares);
    }
    public boolean getUseDriveScaling() { return m_UseDriveScaling; }
    public void setUseDriveScaling(boolean use) {
        m_UseDriveScaling = use;
        this.setMaxOutput(m_UseDriveScaling ? m_DriveScaling : 1.0);
        SmartDashboard.putBoolean("UseDriveScaling", m_UseDriveScaling);
    }
    public double getDriveScaling() { return m_DriveScaling; }
    public void setDriveScaling(double scaling) {
        m_DriveScaling = Math.max(Math.min(scaling, 1.0), 0.1);
        this.setMaxOutput(m_UseDriveScaling ? m_DriveScaling : 1.0);
        SmartDashboard.putNumber("DriveScaling", m_DriveScaling);
    }
    public boolean getQuickTurn() { return m_QuickTurn; }
    public void setQuickTurn(boolean turn) {
        m_QuickTurn = turn;
        SmartDashboard.putBoolean("UseQuickTurn", m_QuickTurn);
    }
    public void toggleDriveMode() {
        switch (m_DriveMode) {
            case ARCADE:    setDriveMode(DriveMode.TANK);       break;
            case TANK:      setDriveMode(DriveMode.CURVATURE);  break;
            case CURVATURE: setDriveMode(DriveMode.ARCADE);     break;
            default:    break;
        }
    }
}