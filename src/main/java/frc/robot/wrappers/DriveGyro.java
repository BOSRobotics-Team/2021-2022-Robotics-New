package frc.robot.wrappers;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveGyro {

  private AHRS ahrs = null;
  private double simAngle = 0.0;
  private double simRate = 0.0;
  private double simHeading = 0.0;

  /** Whether or not to use the NavX for driving straight */
  private boolean overrideGyro = false;

  public DriveGyro(boolean sim) {
    if (!sim) {
      ahrs = new AHRS();
    }
  }

  /**
   * Set the robot's heading.
   *
   * @param heading The heading to set to, in degrees on [-180, 180].
   */
  public void setHeadingDegrees(final double heading) {
    if (ahrs != null) {
      ahrs.setAngleAdjustment(heading + ahrs.getRotation2d().getDegrees());
    } else {
      simHeading += heading;
    }
  }

  /*  SimValue names  -> function
      Connected	    -> isConnected()
      Rate	        -> getRate()
      Yaw	            -> getYaw() or getAngle()
      Pitch	        -> getPitch()
      Roll	        -> getRoll()
      CompassHeading	-> getCompassHeading()
      FusedHeading	-> getFusedHeading()
      LinearWorldAccelX	-> getLinearWorldAccelX()
      LinearWorldAccelY	-> getLinearWorldAccelY()
      LinearWorldAccelZ	-> getLinearWorldAccelZ()
  */
  public void setRawHeadingDegrees(final double heading) {
    if (ahrs != null) {
      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
      angle.set(heading);
    } else {
      simHeading = heading;
    }
  }

  public double getHeadingDegrees() {
    if (ahrs != null) {
      return ahrs.getRotation2d().getDegrees();
    }
    return simHeading;
  }

  public Rotation2d getHeading() {
    if (ahrs != null) {
      return ahrs.getRotation2d();
    }
    return Rotation2d.fromDegrees(simHeading);
  }

  /** Zero the robot's heading. */
  public void zeroHeading() {
    if (ahrs != null) {
      ahrs.reset();
      ahrs.setAngleAdjustment(ahrs.getRotation2d().getDegrees());
    } else {
      simHeading = 0.0;
    }
  }

  /**
   * Get the robot's angular velocity.
   *
   * @return Angular velocity in degrees/sec
   */
  public double getAngularVel() {
    if (ahrs != null) {
      return -ahrs.getRate();
    }
    return -simRate;
  }

  /**
   * Get the robot's angular displacement since being turned on.
   *
   * @return Angular displacement in degrees.
   */
  public double getAngularDisplacement() {
    if (ahrs != null) {
      return -ahrs.getAngle();
    }
    return -simAngle;
  }

  /** @return true if the NavX is currently overriden, false otherwise. */
  public boolean getOverrideGyro() {
    return overrideGyro;
  }

  /** @param override true to override the NavX, false to un-override it. */
  public void setOverrideGyro(final boolean override) {
    overrideGyro = override;
  }

  public void logPeriodic() {
    /* Smart dash plots */
    if (ahrs != null) {
      //          SmartDashboard.putBoolean( "IMU_Connected",        ahrs.isConnected());
      //          SmartDashboard.putBoolean( "IMU_IsCalibrating",    ahrs.isCalibrating());
      SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
      SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
      SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

      /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
      SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

      /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
      //          SmartDashboard.putNumber(  "IMU_FusedHeading",     ahrs.getFusedHeading());

      /* These functions are compatible w/the WPI Gyro Class */
      SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
      SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

      /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
      //          SmartDashboard.putNumber(  "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
      //          SmartDashboard.putNumber(  "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
      SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
      SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

      /* Display estimates of velocity/displacement.  Note that these values are  */
      /* not expected to be accurate enough for estimating robot position on a    */
      /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
      /* of these errors due to single (velocity) integration and especially      */
      /* double (displacement) integration.                                       */
      //          SmartDashboard.putNumber(  "IMU_Temp_C",           ahrs.getTempC());
      //          SmartDashboard.putNumber(  "Velocity_X",           ahrs.getVelocityX() );
      //          SmartDashboard.putNumber(  "Velocity_Y",           ahrs.getVelocityY() );
      //          SmartDashboard.putNumber(  "Displacement_X",       ahrs.getDisplacementX() );
      //          SmartDashboard.putNumber(  "Displacement_Y",       ahrs.getDisplacementY() );

      /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
      /* NOTE:  These values are not normally necessary, but are made available   */
      /* for advanced users.  Before using this data, please consider whether     */
      /* the processed data (see above) will suit your needs.                     */
      //          SmartDashboard.putNumber(   "RawGyro_X",           ahrs.getRawGyroX());
      //          SmartDashboard.putNumber(   "RawGyro_Y",           ahrs.getRawGyroY());
      //          SmartDashboard.putNumber(   "RawGyro_Z",           ahrs.getRawGyroZ());
      //          SmartDashboard.putNumber(   "RawAccel_X",          ahrs.getRawAccelX());
      //          SmartDashboard.putNumber(   "RawAccel_Y",          ahrs.getRawAccelY());
      //          SmartDashboard.putNumber(   "RawAccel_Z",          ahrs.getRawAccelZ());
      //          SmartDashboard.putNumber(   "RawMag_X",            ahrs.getRawMagX());
      //          SmartDashboard.putNumber(   "RawMag_Y",            ahrs.getRawMagY());
      //          SmartDashboard.putNumber(   "RawMag_Z",            ahrs.getRawMagZ());
      //          SmartDashboard.putNumber(   "IMU_Temp_C",          ahrs.getTempC());

      /* Omnimount Yaw Axis Information                                           */
      /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
      //          AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
      //          SmartDashboard.putString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
      //          SmartDashboard.putNumber(  "YawAxis",
      // yaw_axis.board_axis.getValue());

      /* Sensor Board Information                                                 */
      //          SmartDashboard.putString(  "FirmwareVersion",      ahrs.getFirmwareVersion());

      /* Quaternion Data                                                          */
      /* Quaternions are fascinating, and are the most compact representation of  */
      /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
      /* from the Quaternions.  If interested in motion processing, knowledge of  */
      /* Quaternions is highly recommended.                                       */
      //          SmartDashboard.putNumber(  "QuaternionW",          ahrs.getQuaternionW());
      //          SmartDashboard.putNumber(  "QuaternionX",          ahrs.getQuaternionX());
      //          SmartDashboard.putNumber(  "QuaternionY",          ahrs.getQuaternionY());
      //          SmartDashboard.putNumber(  "QuaternionZ",          ahrs.getQuaternionZ());

      /* Connectivity Debugging Support                                           */
      //          SmartDashboard.putNumber(  "IMU_Update_Count",     ahrs.getUpdateCount());
      //          SmartDashboard.putNumber(  "IMU_Byte_Count",       ahrs.getByteCount());
    }
  }
}
