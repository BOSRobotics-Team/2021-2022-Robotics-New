/**
 *  Class that organizes gear ratios used when converting rotations to distances
 */
package frc.robot.util;

public class GearRatios {
    public final double kGearRatio;
    public final double kWheelRadiusInches;
    public final double kPulleyRatio;
	
	public GearRatios(double _gearRatio, double _wheelRadius, double _pulleyRatio) {
		kGearRatio = _gearRatio;
		kWheelRadiusInches = _wheelRadius;
		kPulleyRatio = _pulleyRatio;
	}
}