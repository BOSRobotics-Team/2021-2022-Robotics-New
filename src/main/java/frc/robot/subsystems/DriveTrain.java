package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import frc.robot.util.*;
import frc.robot.util.drivetrains.DifferentialDriveTrain;
import frc.robot.util.drivetrains.DifferentialDriveTrain.DriveMode;

public class DriveTrain extends SubsystemBase {

  public final DifferentialDriveTrain m_differentialDriveTrain;

  public static final double kMaxDriveSpeed = 0.5;

  public DriveTrain() {
    m_differentialDriveTrain =
        new DifferentialDriveTrain(
            Constants.kID_LMasterDrive,
            Constants.kID_RMasterDrive,
            // Constants.kID_LFollowerDrive,
            // Constants.kID_RFollowerDrive,
            Constants.kDriveChassisWidthMeters,
            Constants.kDriveCarriageMass);
    m_differentialDriveTrain.setMaxDriveScaling(kMaxDriveSpeed);

    addChild("Differential Drive", m_differentialDriveTrain);
  }

  public void configDistanceGains(Gains dgains) {
    m_differentialDriveTrain.configDistanceGains(dgains);
  }

  public void configDistanceAndTurnGains(Gains dgains, Gains tgains) {
    m_differentialDriveTrain.configDistanceAndTurnGains(dgains, tgains);
  }

  public void configMotionSCurveStrength(int smoothing) {
    m_differentialDriveTrain.configMotionSCurveStrength(smoothing);
  }

  public void setTarget(double distance) {
    m_differentialDriveTrain.setTarget(distance);
    Shuffleboard.addEventMarker("DriveTrain - setTarget (meters)", EventImportance.kHigh);
    // System.out.println("target (meters) = " + distance);
  }

  public void setTargetAndAngle(double distance, double angle) {
    m_differentialDriveTrain.setTargetAndAngle(distance, angle);
    Shuffleboard.addEventMarker("DriveTrain - setTargetAndAngle", EventImportance.kHigh);
    // System.out.println("target (meters) = " + distance + " angle: " + angle);
  }

  public Boolean isTargetReached() {
    return m_differentialDriveTrain.isTargetReached();
  }

  @Override
  public void periodic() {
    m_differentialDriveTrain.update();
  }

  @Override
  public void simulationPeriodic() {
    m_differentialDriveTrain.simupdate();
  }

  /** Completely stop the robot by setting the voltage to each side to be 0. */
  public void fullStop() {
    m_differentialDriveTrain.fullStop();
  }

  /** Reset odometry tracker to current robot pose */
  public void resetOdometry(final Pose2d pose) {
    m_differentialDriveTrain.resetOdometry(pose);
  }

  /** @return Current estimated pose based on odometry tracker data */
  public Pose2d getCurrentPose() {
    return m_differentialDriveTrain.getCurrentPose();
  }

  /** @return Current wheel speeds based on encoder readings for future pose correction */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return m_differentialDriveTrain.getWheelSpeeds();
  }

  /** @return Kinematics processor for wheel speeds */
  public DifferentialDriveKinematics getDriveKinematics() {
    return m_differentialDriveTrain.getDriveKinematics();
  }

  /** Resets the position of the Talon to 0. */
  public void resetPosition() {
    m_differentialDriveTrain.resetPosition();
  }

  public void logPeriodic() {
    m_differentialDriveTrain.logPeriodic();
  }

  public void enableDriveTrain(boolean enable) {
    m_differentialDriveTrain.enableDriveTrain(enable);
  }

  public void enableBrakes(boolean enabled) {
    m_differentialDriveTrain.enableBrakes(enabled);
  }

  public void zeroHeading() {
    m_differentialDriveTrain.zeroHeading();
  }

  public void driveTank(double leftSpeed, double rightSpeed) {
    m_differentialDriveTrain.driveTank(leftSpeed, rightSpeed);
  }

  public void driveToTarget(double meters) {
    m_differentialDriveTrain.driveToTarget(meters);
  }

  public void drive(XboxController ctrl) {
    m_differentialDriveTrain.drive(
        ctrl.getLeftY(), ctrl.getLeftX(), ctrl.getRightY(), ctrl.getRightX());
  }

  public void setOutput(double left, double right) {
    m_differentialDriveTrain.setOutput(left, right);
  }

  public DriveMode getDriveMode() {
    return m_differentialDriveTrain.getDriveMode();
  }

  public void setDriveMode(DriveMode mode) {
    m_differentialDriveTrain.setDriveMode(mode);
  }

  public void setUseSquares(boolean use) {
    m_differentialDriveTrain.setUseSquares(use);
  }

  public boolean getUseDriveScaling() {
    return m_differentialDriveTrain.getUseDriveScaling();
  }

  public void setUseDriveScaling(boolean use) {
    m_differentialDriveTrain.setUseDriveScaling(use);
  }

  public double getDriveScaling() {
    return m_differentialDriveTrain.getDriveScaling();
  }

  public void setDriveScaling(double scaling) {
    m_differentialDriveTrain.setDriveScaling(scaling);
  }

  public void addDriveScaling(double incr) {
    m_differentialDriveTrain.addDriveScaling(incr);
  }

  public boolean getQuickTurn() {
    return m_differentialDriveTrain.getQuickTurn();
  }

  public void setQuickTurn(boolean turn) {
    m_differentialDriveTrain.setQuickTurn(turn);
  }

  public void toggleDriveMode() {
    m_differentialDriveTrain.toggleDriveMode();
  }
}
