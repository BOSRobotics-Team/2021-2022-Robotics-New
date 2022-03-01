package frc.robot;

import frc.robot.util.*;

public class Constants {

  public static final int kID_RMasterDrive = 0; // TalonFX
  public static final int kID_LMasterDrive = 1; // TalonFX
  public static final int kID_RFollowDrive = 2; // VictorSPX
  public static final int kID_LFollowDrive = 3; // VictorSPX
  public static final int kID_LClimber = 9; // TalonFX
  public static final int kID_RClimber = 10; // TalonFX
  public static final int kID_LPivot = 11; // TalonFX
  public static final int kID_RPivot = 12; // TalonFX
  public static final int kID_Shooter = 18; // VictorSPX
  // public static final int kID_Intake = 5;       //TalonSRX
  // public static final int kID_IntakeLift = 6;   //TalonSRX
  public static final int kID_CANdle = 0;

  public static final double kDriveGearRatio = 10.71;
  public static final double kDriveWheelRadiusInches = 3;
  public static final double kDriveChassisLengthMeters = 1.0;
  public static final double kDriveChassisWidthMeters = 0.8;
  public static final double kDriveCarriageMass = 54.0; // Kilograms

  public static final double kDriveMaxSpeedMetersPerSecond = 5;
  public static final double kDriveMaxAccelerationMetersPerSecondSquared = 2;

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control loop. kF: 1023
   * represents output value to Talon at 100%, 6800 represents Velocity units at 100% output Not all
   * set of Gains are used in this project and may be removed as desired.
   *
   * <p>kP kI kD kF Iz PeakOut
   */
  public static final Gains kDriveGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.80);

  public static final Gains kDriveGains_Turning = new Gains(0.1, 0.0, 0.0, 0.0, 200, 0.75);
  public static final Gains kDriveGains_Velocit =
      new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50);
  public static final Gains kDriveGains_MotProf =
      new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00);

  public static final double kLClimberMaxHeight = 0.56;
  public static final double kRClimberMaxHeight = 0.56;
  public static final GearRatios kClimberGearRatio = new GearRatios(20.0, 0.5, 1.0);
  public static final Gains kClimberGains_Distance =
      new Gains(0.2, 0.0, 0.0, 0., 0, 1.0, 10000, 16000, 1);
  public static final Gains kClimberGains_Turn = new Gains(0.1, 0.0, 0.0, 0., 0, 1.0);
  public static final double kClimberFeedFwd = -0.4;
  public static final double kResetClimberSpeed = -0.15;
  public static final double kResetFastClimberSpeed = -0.20;

  public static final double kLPivotLinkMaxAngle = 105.0;
  public static final double kLPivotLinkMinAngle = 35.0;
  public static final double kRPivotLinkMaxAngle = 105.0;
  public static final double kRPivotLinkMinAngle = 35.0;

  public static final GearRatios kPivotLinkGearRatio =
      new GearRatios(100.0, Convertor.kWheelRadiusForDegrees, 1.0);
  public static final Gains kPivotLinkGains_Distance =
      new Gains(0.2, 0.0, 0.0, 0.2, 0, 0.2, 2000, 2000, 0);
  public static final Gains kPivotLinkGains_Turn = new Gains(0.1, 0.0, 0.0, 0.1, 0, 0.3);
  public static final double kPivotLinkMass = 3.0; // Kilograms
  public static final double kPivotLinkLength = 0.76; // meters
  public static final double kPivotLinkFeedFwd = 0.0;
  public static final double kResetPivotSpeed = 0.05;
  public static final double kResetFastPivotSpeed = 0.1;

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
