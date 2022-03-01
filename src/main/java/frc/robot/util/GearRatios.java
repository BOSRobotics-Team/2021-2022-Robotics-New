/** Class that organizes gear ratios used when converting rotations to distances */
package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class GearRatios {
  public static final double kWheelRadiusForRadians = Units.metersToInches(1);
  public static final double kWheelRadiusForDegrees = Units.metersToInches(180.0 / Math.PI);

  public final double kGearRatio;
  public final double kWheelRadiusInches;
  public final double kPulleyRatio;

  public GearRatios(double gearRatio, double wheelRadius, double pulleyRatio) {
    kGearRatio = gearRatio;
    kWheelRadiusInches = wheelRadius;
    kPulleyRatio = pulleyRatio;
  }

  public GearRatios(double gearRatio, double wheelRadius) {
    this(gearRatio, wheelRadius, 1.0);
  }

  public GearRatios(double gearRatio) {
    this(gearRatio, kWheelRadiusForRadians, 1.0);
  }
}
