package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class Convertor {
    public static final int k100msPerSecond = 10;

    public int _countsPerRev = 2048;

    public double _metersToUnitsFactor = _countsPerRev / (2. * Math.PI);
    public double _velToUnitsFactor = _metersToUnitsFactor / k100msPerSecond;
    public double _unitsToMetersFactor = (2. * Math.PI) / _countsPerRev;
    public double _unitsToVelFactor = _unitsToMetersFactor * k100msPerSecond;

    public Convertor( int countsPerRev ) {
        _countsPerRev = countsPerRev;
    }
    public Convertor( int countsPerRev, GearRatios gearRatio ) {
        _countsPerRev = countsPerRev;
        this.setRatios(gearRatio);
    }

    public void setRatios(GearRatios ratios) {
        _metersToUnitsFactor = _countsPerRev * ratios.kPulleyRatio * ratios.kGearRatio / (2. * Math.PI * Units.inchesToMeters(ratios.kWheelRadiusInches));
        _velToUnitsFactor = _metersToUnitsFactor / k100msPerSecond;

        _unitsToMetersFactor = (2. * Math.PI * Units.inchesToMeters(ratios.kWheelRadiusInches)) / (_countsPerRev * ratios.kPulleyRatio * ratios.kGearRatio);
        _unitsToVelFactor = _unitsToMetersFactor * k100msPerSecond;
    }
    public void setRatios(double gearRatio, double wheelRadiusInches, double pulleyRatio) {
        this.setRatios(new GearRatios(gearRatio, wheelRadiusInches, pulleyRatio));
    }

    public int distanceMetersToNativeUnits(double positionMeters) {
        return (int)(positionMeters * _metersToUnitsFactor);
    }
    public double nativeUnitsToDistanceMeters(double sensorCounts) {
        return (double)sensorCounts * _unitsToMetersFactor;
    }
    public int velocityToNativeUnits(double velocityMetersPerSecond) {
        return (int)(velocityMetersPerSecond * _velToUnitsFactor);
    }    
    public double nativeUnitsToVelocity(double sensorCountsPer100ms) {
        return (double)(sensorCountsPer100ms * _unitsToMetersFactor);
    }  
}
