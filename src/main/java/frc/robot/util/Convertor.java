package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class Convertor {
  public static final int k100msPerSecond = 10;
  public static final double k2PI = 2.0 * Math.PI;
  public static final double kWheelRadiusForRadians = Units.metersToInches(1);
  public static final double kWheelRadiusForDegrees = Units.metersToInches(360.0 / k2PI);

  public int unitsPerRevolution = 2048;
  public GearRatios gearRatios;

  public double _metersToUnitsFactor = unitsPerRevolution / (2. * Math.PI);
  public double _velToUnitsFactor = _metersToUnitsFactor / k100msPerSecond;
  public double _unitsToMetersFactor = (2. * Math.PI) / unitsPerRevolution;
  public double _unitsToVelFactor = _unitsToMetersFactor * k100msPerSecond;
  public double _wheelCircumferenceMeters = Units.inchesToMeters(k2PI * kWheelRadiusForRadians);

  public Convertor(int unitsPerRev, GearRatios ratios) {
    unitsPerRevolution = unitsPerRev;
    gearRatios = ratios;
    this.setRatios(gearRatios);
  }

  public Convertor(int unitsPerRev) {
    this(unitsPerRev, new GearRatios(1.0, kWheelRadiusForRadians, 1.0));
  }

  public void setRatios(GearRatios ratios) {
    gearRatios = ratios;
    _wheelCircumferenceMeters = Units.inchesToMeters(k2PI * gearRatios.kWheelRadiusInches);
    _metersToUnitsFactor =
        unitsPerRevolution
            * gearRatios.kPulleyRatio
            * gearRatios.kGearRatio
            / _wheelCircumferenceMeters;
    _velToUnitsFactor = _metersToUnitsFactor / k100msPerSecond;

    _unitsToMetersFactor =
        _wheelCircumferenceMeters
            / (unitsPerRevolution * gearRatios.kPulleyRatio * gearRatios.kGearRatio);
    _unitsToVelFactor = _unitsToMetersFactor * k100msPerSecond;
  }

  public void setRatios(double gearRatio, double wheelRadiusInches, double pulleyRatio) {
    this.setRatios(new GearRatios(gearRatio, wheelRadiusInches, pulleyRatio));
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
