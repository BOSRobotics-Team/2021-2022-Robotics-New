// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import frc.robot.util.*;
import frc.robot.wrappers.*;

public class DriveTrain extends SubsystemBase {

  public enum DriveMode {
    ARCADE,
    TANK,
    CURVATURE
  }

  private final WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.kID_RMasterDrive);
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.kID_LMasterDrive);
  private final WPI_VictorSPX rightFollower = new WPI_VictorSPX(Constants.kID_RFollowDrive);
  private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(Constants.kID_LFollowDrive);

  private final TalonFXSimCollection leftMasterSim = leftMaster.getSimCollection();
  private final TalonFXSimCollection rightMasterSim = rightMaster.getSimCollection();

  public final SmartMotorController smartController =
      new SmartMotorController(leftMaster, rightMaster, "DriveTrain");

  public final DifferentialDrive differentialDrive;

  private final Field2d m_field = new Field2d();

  /** The NavX gyro */
  private final DriveGyro gyro = new DriveGyro(true);

  /** Drivetrain odometry tracker for tracking position */
  private final DifferentialDriveOdometry driveOdometry =
      new DifferentialDriveOdometry(gyro.getHeading());

  /** Drivetrain kinematics processor for measuring individual wheel speeds */
  private final DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(Constants.kWidthChassisMeters);

  /* Simulation model of the drivetrain */
  private final DifferentialDrivetrainSim m_driveSim =
      new DifferentialDrivetrainSim(
          DCMotor.getFalcon500(2), // 2 CIMS on each side of the drivetrain.
          SmartMotorController.kDefaultGearRatio.kGearRatio, // Standard AndyMark Gearing reduction.
          2.1, // MOI of 2.1 kg m^2 (from CAD model).
          26.5, // Mass of the robot is 26.5 kg.
          Units.inchesToMeters(
              SmartMotorController.kDefaultGearRatio
                  .kWheelRadiusInches), // Robot uses 3" radius (6" diameter) wheels.
          Constants.kWidthChassisMeters, // Distance between wheels is _ meters.
          null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
          );

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
    smartController.configureRatios(SmartMotorController.kDefaultGearRatio);
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
    resetPosition();

    addChild("Differential Drive", differentialDrive);

    SmartDashboard.putData("Field2d", m_field);
  }

  public void configForPID() {
    smartController.setDistanceConfigs(Constants.kGains_Distanc);
  }

  public void configForPID2() {
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
    System.out.println("target (meters) = " + distance);
  }

  public void setTarget(double distance, double angle) {
    smartController.setTarget(distance, angle);
    differentialDrive.feed();
    System.out.println("target (meters) = " + distance + " angle: " + angle);
  }

  public Boolean isTargetReached() {
    differentialDrive.feed();
    return smartController.isTargetFinished();
  }

  @Override
  public void periodic() {
    smartController.update();
    updateOdometry();
  }

  @Override
  public void simulationPeriodic() {

    m_driveSim.setInputs(
        leftMasterSim.getMotorOutputLeadVoltage(), -rightMasterSim.getMotorOutputLeadVoltage());

    /*
     * Advance the model by 20 ms. Note that if you are running this
     * subsystem in a separate thread or have changed the nominal
     * timestep of TimedRobot, this value needs to match it.
     */
    m_driveSim.update(0.02);

    // Update all of our sensors.
    leftMasterSim.setIntegratedSensorRawPosition(
        smartController.MetersToUnits(m_driveSim.getLeftPositionMeters()));
    rightMasterSim.setIntegratedSensorRawPosition(
        smartController.MetersToUnits(m_driveSim.getRightPositionMeters()));
    leftMasterSim.setIntegratedSensorVelocity(
        smartController.VelocityToUnits(m_driveSim.getLeftVelocityMetersPerSecond()));
    rightMasterSim.setIntegratedSensorVelocity(
        smartController.VelocityToUnits(m_driveSim.getRightVelocityMetersPerSecond()));

    gyro.setRawHeadingDegrees(m_driveSim.getHeading().getDegrees());

    // Update other inputs to Talons
    leftMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
  }

  public Double getVelocity() {
    return smartController.getVelocity();
  }

  public Double getPosition() {
    return smartController.getDistance();
  }

  public Double getNativePosition() {
    return smartController.getPosition();
  }

  public Double getAuxPosition() {
    return smartController.getAuxDistance();
  }

  public Double getAuxNativePosition() {
    return smartController.getAuxPosition();
  }

  /**
   * Get the velocity of the left side of the drive.
   *
   * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
   */
  public Double getLeftVel() {
    return smartController.getLeftVelocity();
  }

  /**
   * Get the velocity of the right side of the drive.
   *
   * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
   */
  public Double getRightVel() {
    return smartController.getRightVelocity();
  }

  /**
   * Get the position of the left side of the drive.
   *
   * @return The signed position in feet, or null if the drive doesn't have encoders.
   */
  public Double getLeftDistance() {
    return smartController.getLeftDistance();
  }

  /**
   * Get the position of the right side of the drive.
   *
   * @return The signed position in feet, or null if the drive doesn't have encoders.
   */
  public Double getRightDistance() {
    return smartController.getRightDistance();
  }

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
    //        double angle = ((getRightDistance() - getLeftDistance()) * (180.0 / Math.PI)) /
    // Constants.kWidthChassisMeters;

    driveOdometry.update(
        getHeading(), /// *Rotation2d.fromDegrees(angle),
        getLeftDistance(),
        getRightDistance());
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
    driveOdometry.resetPosition(m_field.getRobotPose(), getHeading());
  }

  public void logPeriodic() {

    gyro.logPeriodic();
    // smartController.logPeriodic();

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
   *
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
      this.setOutput(-ctrl.getLeftY(), ctrl.getRightX());
    } else if (m_DriveMode == DriveMode.TANK) {
      this.setOutput(-ctrl.getLeftY(), -ctrl.getRightY());
    } else if (m_DriveMode == DriveMode.CURVATURE) {
      this.setOutput(-ctrl.getLeftY(), ctrl.getRightX());
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

  public DriveMode getDriveMode() {
    return m_DriveMode;
  }

  public void setDriveMode(DriveMode mode) {
    m_DriveMode = mode;
    SmartDashboard.putString("DriveTrainMode", m_DriveMode.toString());
  }

  public boolean getUseSquares() {
    return m_UseSquares;
  }

  public void setUseSquares(boolean use) {
    m_UseSquares = use;
    SmartDashboard.putBoolean("UseSquares", m_UseSquares);
  }

  public boolean getUseDriveScaling() {
    return m_UseDriveScaling;
  }

  public void setUseDriveScaling(boolean use) {
    m_UseDriveScaling = use;
    this.setMaxOutput(m_UseDriveScaling ? m_DriveScaling : 1.0);
    SmartDashboard.putBoolean("UseDriveScaling", m_UseDriveScaling);
  }

  public double getDriveScaling() {
    return m_DriveScaling;
  }

  public void setDriveScaling(double scaling) {
    m_DriveScaling = Math.max(Math.min(scaling, 1.0), 0.1);
    this.setMaxOutput(m_UseDriveScaling ? m_DriveScaling : 1.0);
    SmartDashboard.putNumber("DriveScaling", m_DriveScaling);
  }

  public void addDriveScaling(double incr) {
    setDriveScaling(m_DriveScaling + incr);
  }

  public boolean getQuickTurn() {
    return m_QuickTurn;
  }

  public void setQuickTurn(boolean turn) {
    m_QuickTurn = turn;
    SmartDashboard.putBoolean("UseQuickTurn", m_QuickTurn);
  }

  public void toggleDriveMode() {
    switch (m_DriveMode) {
      case ARCADE:
        setDriveMode(DriveMode.TANK);
        break;
      case TANK:
        setDriveMode(DriveMode.CURVATURE);
        break;
      case CURVATURE:
        setDriveMode(DriveMode.ARCADE);
        break;
      default:
        break;
    }
  }
}
