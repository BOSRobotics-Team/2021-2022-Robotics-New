package frc.robot.util.drivetrains;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.*;
import frc.robot.util.sim.*;
import frc.robot.util.wrappers.*;

public class DifferentialDriveTrain implements Sendable {

  public enum DriveMode {
    ARCADE,
    TANK,
    CURVATURE
  }

  private static final double kDefaultMaxDriveSpeed = 0.5;

  private final WPI_TalonFX leftMaster;
  private final WPI_TalonFX rightMaster;
  private WPI_TalonFX slideMaster = null;
  private WPI_TalonFX leftFollower = null;
  private WPI_TalonFX rightFollower = null;
  private WPI_TalonFX slideFollower = null;

  public final SmartMotor leftController;
  public final SmartMotor rightController;
  public SmartMotor slideController = null;

  public final DifferentialDrive differentialDrive;

  private final Field2d m_field = new Field2d();

  /** The NavX gyro */
  private final DriveGyro gyro = new DriveGyro(true);

  /** Drivetrain odometry tracker for tracking position */
  private final SlideDriveOdometry driveOdometry;

  /** Drivetrain kinematics processor for measuring individual wheel speeds */
  private final DifferentialDriveKinematics driveKinematics;

  /** Drivetrain simulator */
  private final DrivetrainSim m_drivetrainSim;

  private DriveMode m_DriveMode = DriveMode.ARCADE;
  private boolean m_UseSquares = true;
  private boolean m_UseDriveScaling = false;
  private double m_MaxDriveScaling = kDefaultMaxDriveSpeed;
  private double m_DriveScaling = kDefaultMaxDriveSpeed;
  private boolean m_QuickTurn = false;

  private double _lastLSmoothing = 0.0;
  private double _lastRSmoothing = 0.0;

  public DifferentialDriveTrain(
      int leftID,
      int rightID,
      int slideID,
      int leftFollowID,
      int rightFollowID,
      int slideFollowID,
      double chassisWidth,
      double chassisWeight) {

    leftMaster = new WPI_TalonFX(leftID);
    leftMaster.configFactoryDefault();
    leftMaster.setInverted(InvertType.None);
    if (leftFollowID >= 0) {
      leftFollower = new WPI_TalonFX(leftFollowID);
      leftFollower.configFactoryDefault();
      leftFollower.setInverted(InvertType.FollowMaster);
      leftFollower.follow(leftMaster);
    }
    leftController = new SmartMotor(leftMaster, "Left Drivetrain");
    leftController.initController();
    leftController.configureRatios(SmartMotor.kDefaultGearRatio);
    leftController.enableBrakes(false);

    rightMaster = new WPI_TalonFX(rightID);
    rightMaster.configFactoryDefault();
    rightMaster.setInverted(InvertType.InvertMotorOutput);
    if (rightFollowID >= 0) {
      rightFollower = new WPI_TalonFX(rightFollowID);
      rightFollower.configFactoryDefault();
      rightFollower.setInverted(InvertType.FollowMaster);
      rightFollower.follow(rightMaster);
    }
    rightController = new SmartMotor(rightMaster, "Right Drivetrain");
    rightController.initController();
    rightController.configureRatios(SmartMotor.kDefaultGearRatio);
    rightController.enableBrakes(false);

    if (slideID >= 0) {
      slideMaster = new WPI_TalonFX(slideID);
      slideMaster.configFactoryDefault();
      slideMaster.setInverted(InvertType.None);
      if (slideFollowID >= 0) {
        slideFollower = new WPI_TalonFX(slideFollowID);
        slideFollower.configFactoryDefault();
        slideFollower.setInverted(InvertType.FollowMaster);
        slideFollower.follow(slideMaster);
      }
      slideController = new SmartMotor(slideMaster);
      slideController.initController();
      slideController.configureRatios(SmartMotor.kDefaultGearRatio);
      slideController.enableBrakes(false);
    }

    differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
    differentialDrive.setSafetyEnabled(false);
    differentialDrive.setExpiration(0.1);
    differentialDrive.setMaxOutput(m_MaxDriveScaling);
    differentialDrive.setDeadband(0.02);

    driveOdometry = new SlideDriveOdometry(gyro.getHeading());
    driveKinematics = new DifferentialDriveKinematics(chassisWidth);
    m_drivetrainSim =
        new DrivetrainSim(
            leftController,
            rightController,
            gyro,
            chassisWeight, // 23.0, // 53.0,
            chassisWidth);

    this.resetPosition();
    SmartDashboard.putData("Field2d", m_field);
  }

  public DifferentialDriveTrain(
      int leftID,
      int rightID,
      int leftFollowID,
      int rightFollowID,
      double chassisWidth,
      double chassisWeight) {
    this(leftID, rightID, -1, leftFollowID, rightFollowID, -1, chassisWidth, chassisWeight);
  }

  public DifferentialDriveTrain(
      int leftID, int rightID, double chassisWidth, double chassisWeight) {
    this(leftID, rightID, -1, -1, -1, -1, chassisWidth, chassisWeight);
  }

  public void configDistanceGains(Gains dgains) {
    leftController.setDistanceConfigs(dgains, rightController.getDeviceID());
  }

  public void configDistanceAndTurnGains(Gains dgains, Gains tgains) {
    leftController.setDistanceAndTurnConfigs(dgains, tgains, rightController.getDeviceID());
  }

  public void configMotionSCurveStrength(int smoothing) {
    leftMaster.configMotionSCurveStrength(smoothing);
    rightMaster.configMotionSCurveStrength(smoothing);
    SmartDashboard.putNumber("Smoothing", smoothing);
  }

  public void setTarget(double distance) {
    leftController.setTarget(distance);
    differentialDrive.setSafetyEnabled(false);
  }

  public void setTargetAndAngle(double distance, double angle) {
    leftController.setTargetAndAngle(distance, angle);
    differentialDrive.setSafetyEnabled(false);
  }

  public Boolean isTargetReached() {
    differentialDrive.setSafetyEnabled(true);
    return leftController.isTargetFinished();
  }

  public void update() {
    leftController.update();
    rightController.update();
  }

  public void simupdate() {
    updateOdometry();
    m_drivetrainSim.run();
  }

  public Double getVelocity() {
    return leftController.getVelocity();
  }

  public Double getDistance() {
    return leftController.getDistance();
  }

  public Double getPosition() {
    return leftController.getPosition();
  }

  public Double getTurnDistance() {
    return leftController.getTurnDistance();
  }

  public Double getTurnPosition() {
    return leftController.getTurnPosition();
  }

  /**
   * Get the velocity of the left side of the drive.
   *
   * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
   */
  public Double getLeftVelocity() {
    return leftController.getVelocity();
  }

  /**
   * Get the velocity of the right side of the drive.
   *
   * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
   */
  public Double getRightVelocity() {
    return rightController.getVelocity();
  }

  /**
   * Get the position of the left side of the drive.
   *
   * @return The signed position in feet, or null if the drive doesn't have encoders.
   */
  public Double getLeftDistance() {
    return leftController.getDistance();
  }

  /**
   * Get the position of the right side of the drive.
   *
   * @return The signed position in feet, or null if the drive doesn't have encoders.
   */
  public Double getRightDistance() {
    return rightController.getDistance();
  }

  /** Completely stop the robot by setting the voltage to each side to be 0. */
  public void fullStop() {
    this.setPercentVoltage(0, 0);
    if (slideMaster != null) slideMaster.setVoltage(0);
    _lastLSmoothing = _lastRSmoothing = 0.0;
  }

  /** Reset odometry tracker to current robot pose */
  public void resetOdometry(final Pose2d pose) {
    this.resetPosition();
    this.setHeadingDegrees(pose.getRotation().getDegrees());
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
        getRightDistance(),
        (slideController != null) ? slideController.getDistance() : 0.0);
    m_field.setRobotPose(driveOdometry.getPoseMeters());

    SmartDashboard.putData("Field2d", m_field);
    SmartDashboard.putString("Heading", driveOdometry.getPoseMeters().getRotation().toString());
  }

  /** @return Current estimated pose based on odometry tracker data */
  public Pose2d getCurrentPose() {
    return driveOdometry.getPoseMeters() != null ? driveOdometry.getPoseMeters() : new Pose2d();
  }

  /** @return Current wheel speeds based on encoder readings for future pose correction */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  /** @return Kinematics processor for wheel speeds */
  public DifferentialDriveKinematics getDriveKinematics() {
    return driveKinematics;
  }

  /** Disable the motors. */
  public void disable() {
    leftMaster.disable();
    rightMaster.disable();
    if (slideMaster != null) slideMaster.disable();
  }

  public void setPercentVoltage(double leftPctVolts, double rightPctVolts) {
    leftMaster.set(ControlMode.PercentOutput, leftPctVolts);
    rightMaster.set(ControlMode.PercentOutput, rightPctVolts);
  }

  /** Resets the position of the Talon to 0. */
  public void resetPosition() {
    leftController.resetSensorPosition();
    rightController.resetSensorPosition();
    driveOdometry.resetPosition(m_field.getRobotPose(), getHeading());
  }

  public void logPeriodic() {
    gyro.logPeriodic();
    leftController.logPeriodic();
    rightController.logPeriodic();
  }

  public void enableDriveTrain(boolean enable) {
    differentialDrive.setSafetyEnabled(enable);

    this.setPercentVoltage(0.0, 0.0);
    if (!enable) {
      leftMaster.neutralOutput();
      rightMaster.neutralOutput();
    }
  }

  public void enableBrakes(boolean enabled) {
    leftController.enableBrakes(enabled);
    rightController.enableBrakes(enabled);
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
    leftController.setTarget(meters);
    differentialDrive.feed();
  }

  public void drive(double leftY, double leftX, double rightY, double rightX) {
    switch (m_DriveMode) {
      case ARCADE:
        this.setOutput(-leftY, rightX, leftX);
        break;
      case TANK:
        this.setOutput(-leftY, -rightY, leftX);
        break;
      case CURVATURE:
        this.setOutput(-leftY, rightX, leftX);
        break;
    }
  }

  public void setOutput(double left, double right, double slide) {
    double newleft = (_lastLSmoothing + left) / 2.0;
    double newRight = (_lastRSmoothing + right) / 2.0;
    _lastLSmoothing = left;
    _lastRSmoothing = right;

    switch (m_DriveMode) {
      case ARCADE:
        this.driveArcade(newleft, newRight, m_UseSquares);
        break;
      case TANK:
        this.driveTank(newleft, newRight);
        break;
      case CURVATURE:
        this.driveCurvature(newleft, newRight, m_QuickTurn);
        break;
    }
    if (slideMaster != null) {
      slideMaster.set(ControlMode.PercentOutput, slide * m_MaxDriveScaling);
    }
  }

  public void setOutput(double left, double right) {
    this.setOutput(left, right, 0.0);
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
    this.setMaxOutput(m_UseDriveScaling ? m_DriveScaling : m_MaxDriveScaling);
    SmartDashboard.putBoolean("UseDriveScaling", m_UseDriveScaling);
  }

  public double getDriveScaling() {
    return m_DriveScaling;
  }

  public void setDriveScaling(double scaling) {
    m_DriveScaling = Math.max(Math.min(scaling, 1.0), 0.05);
    this.setMaxOutput(m_UseDriveScaling ? m_DriveScaling : m_MaxDriveScaling);
    SmartDashboard.putNumber("DriveScaling", m_DriveScaling);
  }

  public void setMaxDriveScaling(double scaling) {
    m_MaxDriveScaling = Math.max(Math.min(scaling, 1.0), 0.05);
    this.setDriveScaling(m_DriveScaling);
  }

  public void addDriveScaling(double incr) {
    this.setDriveScaling(m_DriveScaling + incr);
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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DifferentialDrive");
    builder.setActuator(true);
    builder.setSafeState(this::fullStop);
    builder.addDoubleProperty("Left Motor Speed", leftMaster::get, leftMaster::set);
    builder.addDoubleProperty(
        "Right Motor Speed", () -> rightMaster.get(), x -> rightMaster.set(x));
  }
}
