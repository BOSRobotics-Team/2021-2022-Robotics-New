/** Class that organizes gains used when assigning values to slots */
package frc.robot.util;

public class Gains {
  public final double kP;
  public final double kI;
  public final double kD;
  public final double kF;
  public final int kIzone;
  public final double kPeakOutput;
  public final double kMotionAccel;
  public final double kMotionCruiseVel;
  public final int kMotionSCurve;

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control loop.
   *
   * <p>_kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100%
   * output Not all set of Gains are used in this project and may be removed as desired.
   */
  public Gains(
      double _kP,
      double _kI,
      double _kD,
      double _kF,
      int _kIzone,
      double _kPeakOutput,
      double motAccel,
      double motVel,
      int motS) {
    kP = _kP;
    kI = _kI;
    kD = _kD;
    kF = _kF;
    kIzone = _kIzone;
    kPeakOutput = _kPeakOutput;
    kMotionAccel = motAccel;
    kMotionCruiseVel = motVel;
    kMotionSCurve = motS;
  }

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control loop.
   *
   * <p>_kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100%
   * output Not all set of Gains are used in this project and may be removed as desired.
   */
  public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) {
    this(_kP, _kI, _kD, _kF, _kIzone, _kPeakOutput, 6000, 15000, 0);
  }
}
