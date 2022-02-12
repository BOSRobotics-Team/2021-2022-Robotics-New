package frc.robot;

import frc.robot.util.*;

public class Constants {

  public static final int kID_RMasterDrive = 0; // TalonFX
  public static final int kID_LMasterDrive = 1; // TalonFX
  public static final int kID_RFollowDrive = 3; // VictorSPX
  public static final int kID_LFollowDrive = 2; // VictorSPX
  public static final int kID_LClimber = 9; // TalonFX
  public static final int kID_RClimber = 10; // TalonFX
  public static final int kID_LPivot = 11; // TalonFX
  public static final int kID_RPivot = 12; // TalonFX
  public static final int kID_Shooter = 15; // VictorSPX
  // public static final int kID_Intake = 5;       //TalonSRX
  // public static final int kID_IntakeLift = 6;   //TalonSRX
  public static final int kID_CANdle = 0;

  public static final double kGearRatio = 10.71;
  public static final double kWheelRadiusInches = 3;
  public static final double kLengthChassisMeters = 1.0;
  public static final double kWidthChassisMeters = 0.8;

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control loop. kF: 1023
   * represents output value to Talon at 100%, 6800 represents Velocity units at 100% output Not all
   * set of Gains are used in this project and may be removed as desired.
   *
   * <p>kP kI kD kF Iz PeakOut
   */
  public static final Gains kGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.80);

  public static final Gains kGains_Turning = new Gains(0.1, 0.0, 0.0, 0.0, 200, 0.75);
  public static final Gains kGains_Velocit = new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50);
  public static final Gains kGains_MotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00);

  public static final double kMaxSpeedMetersPerSecond = 5;
  public static final double kMaxAccelerationMetersPerSecondSquared = 2;

  // These characterization values MUST be determined either experimentally or theoretically
  // for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
  // values for your robot.
  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;
  public static final double kPDriveVel = 8.5;
  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
}
