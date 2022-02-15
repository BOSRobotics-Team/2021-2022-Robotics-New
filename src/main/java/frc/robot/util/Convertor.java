package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class Convertor {
  public static final int k100msPerSecond = 10;
  public static final double k2PI = 2.0 * Math.PI;
  public static final double kWheelRadiusForRadians = Units.metersToInches(1);
  public static final double kWheelRadiusForDegrees = Units.metersToInches(360.0 / k2PI);

  public int _countsPerRev = 2048;

  public double _metersToUnitsFactor = _countsPerRev / (2. * Math.PI);
  public double _velToUnitsFactor = _metersToUnitsFactor / k100msPerSecond;
  public double _unitsToMetersFactor = (2. * Math.PI) / _countsPerRev;
  public double _unitsToVelFactor = _unitsToMetersFactor * k100msPerSecond;

  public Convertor(int countsPerRev) {
    _countsPerRev = countsPerRev;
  }

  public Convertor(int countsPerRev, GearRatios gearRatio) {
    _countsPerRev = countsPerRev;
    this.setRatios(gearRatio);
  }

  public void setRatios(double gearRatio, double wheelRadiusInches, double pulleyRatio) {
    double wheelCircumferenceMeters = Units.inchesToMeters(k2PI * wheelRadiusInches);
    _metersToUnitsFactor = _countsPerRev * pulleyRatio * gearRatio / wheelCircumferenceMeters;
    _velToUnitsFactor = _metersToUnitsFactor / k100msPerSecond;

    _unitsToMetersFactor = wheelCircumferenceMeters / (_countsPerRev * pulleyRatio * gearRatio);
    _unitsToVelFactor = _unitsToMetersFactor * k100msPerSecond;
  }

  public void setRatios(GearRatios ratios) {
    this.setRatios(ratios.kGearRatio, ratios.kWheelRadiusInches, ratios.kPulleyRatio);
  }

  public int distanceMetersToNativeUnits(double positionMeters) {
    return (int) (positionMeters * _metersToUnitsFactor);
  }

  public double nativeUnitsToDistanceMeters(double sensorCounts) {
    return (double) sensorCounts * _unitsToMetersFactor;
  }

  public int velocityToNativeUnits(double velocityMetersPerSecond) {
    return (int) (velocityMetersPerSecond * _velToUnitsFactor);
  }

  public double nativeUnitsToVelocity(double sensorCountsPer100ms) {
    return (double) (sensorCountsPer100ms * _unitsToMetersFactor);
  }
}
