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

  public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) {
    this(_kP, _kI, _kD, _kF, _kIzone, _kPeakOutput, 6000, 15000, 0);
  }
}
