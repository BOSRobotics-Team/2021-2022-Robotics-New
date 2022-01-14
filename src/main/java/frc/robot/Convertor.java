package frc.robot;

import edu.wpi.first.math.util.Units;

public class Convertor {
    public static final int k100msPerSecond = 10;

    public int _countsPerRev = 2048;
    public double _gearRatio = Constants.kGearRatio;
    public double _wheelRadiusInches = Constants.kWheelRadiusInches;

    public double _metersToUnitsFactor = _countsPerRev * _gearRatio / (2. * Math.PI * Units.inchesToMeters(_wheelRadiusInches));
    public double _unitsToMetersFactor = (2. * Math.PI * Units.inchesToMeters(_wheelRadiusInches)) / (_countsPerRev * _gearRatio);
    public double _velToUnitsFactor = _metersToUnitsFactor / k100msPerSecond;
    public double _unitsToVelFactor = _unitsToMetersFactor * k100msPerSecond;

    public Convertor( int countsPerRev, double gearRatio, double wheelRadius ) {
        _countsPerRev = countsPerRev;
        this.setRatios(gearRatio, wheelRadius);
    }

    public void setRatios(double gearRatio, double wheelRadius) {
        _gearRatio = gearRatio;
        _wheelRadiusInches = wheelRadius;

        _metersToUnitsFactor = _countsPerRev * _gearRatio / (2. * Math.PI * Units.inchesToMeters(_wheelRadiusInches));
        _unitsToMetersFactor = (2. * Math.PI * Units.inchesToMeters(_wheelRadiusInches)) / (_countsPerRev * _gearRatio);
        _velToUnitsFactor = _metersToUnitsFactor / k100msPerSecond;
        _unitsToVelFactor = _unitsToMetersFactor * k100msPerSecond;
    }

    public int distanceMetersToNativeUnits(double positionMeters) {
//        double wheelRotations = positionMeters / (2. * Math.PI * Units.inchesToMeters(_wheelRadiusInches));
//        double motorRotations = wheelRotations * _gearRatio;
//        int sensorCounts = (int)(motorRotations * _countsPerRev);
//        return sensorCounts;
        return (int)(positionMeters * _metersToUnitsFactor);
    }
    public double nativeUnitsToDistanceMeters(double sensorCounts) {
//        double motorRotations = (double)sensorCounts / _countsPerRev;
//        double wheelRotations = motorRotations / _gearRatio;
//        double positionMeters = wheelRotations * (2. * Math.PI * Units.inchesToMeters(_wheelRadiusInches));
//        return positionMeters;
        return (double)sensorCounts * _unitsToMetersFactor;
    }

    public int velocityToNativeUnits(double velocityMetersPerSecond) {
//        double wheelRotationsPerSecond = velocityMetersPerSecond / (2. * Math.PI * Units.inchesToMeters(_wheelRadiusInches));
//        double motorRotationsPerSecond = wheelRotationsPerSecond * _gearRatio;
//        double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
//        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * _countsPerRev);
//        return sensorCountsPer100ms;

        return (int)(velocityMetersPerSecond * _velToUnitsFactor);
    }    
    public double nativeUnitsToVelocity(double sensorCountsPer100ms) {
//        double motorRotationsPer100ms = (double)sensorCountsPer100ms / _countsPerRev;
//        double motorRotationsPerSecond = motorRotationsPer100ms * k100msPerSecond;
//        double wheelRotationsPerSecond = motorRotationsPerSecond / _gearRatio;
//        double velocityMetersPerSecond = wheelRotationsPerSecond * (2. * Math.PI * Units.inchesToMeters(_wheelRadiusInches));
//        return velocityMetersPerSecond;

        return (double)(sensorCountsPer100ms * _unitsToMetersFactor);
    }  
}
