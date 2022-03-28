package frc.robot.util.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.*;
import frc.robot.util.*;
import frc.robot.util.SmartMotor.ControllerTypes;
import frc.robot.util.wrappers.*;

public class DrivetrainSim {
  private final SmartMotor leftMaster;
  private final SmartMotor rightMaster;

  /** The NavX gyro */
  private final DriveGyro gyro;

  private final Convertor m_driveConvertor;
  private final double m_driveMass;
  private final double m_driveWidth;

  /* Simulation model of the drivetrain */
  private final DifferentialDrivetrainSim m_driveSim;

  private double m_lMotorOutputVoltage = 0.0;
  private double m_rMotorOutputVoltage = 0.0;
  private int m_lPosition = 0;
  private int m_rPosition = 0;
  private int m_lVelocity = 0;
  private int m_rVelocity = 0;

  public DrivetrainSim(
      SmartMotor left, SmartMotor right, DriveGyro gy, double driveMass, double driveWidth) {
    leftMaster = left;
    rightMaster = right;

    gyro = gy;
    m_driveConvertor = leftMaster.getConvertor();
    m_driveMass = driveMass;
    m_driveWidth = driveWidth;

    m_driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2), // 2 CIMS on each side of the drivetrain.
            m_driveConvertor.gearRatios.kGearRatio, // Standard AndyMark Gearing reduction.
            2.1, // MOI of 2.1 kg m^2 (from CAD model).
            m_driveMass, // 26.5, // Mass of the robot is 26.5 kg.
            Units.inchesToMeters(
                m_driveConvertor.gearRatios.kWheelRadiusInches), // 3" radius wheels.
            m_driveWidth, // Distance between wheels is _ meters.
            null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
            );
  }

  public void run() {

    if (leftMaster.getControllerType() == ControllerTypes.TalonFX)
      m_lMotorOutputVoltage = leftMaster.getFXSimCollection().getMotorOutputLeadVoltage();
    else if (leftMaster.getControllerType() == ControllerTypes.TalonSRX)
      m_lMotorOutputVoltage = leftMaster.getSRXSimCollection().getMotorOutputLeadVoltage();
    if (rightMaster.getControllerType() == ControllerTypes.TalonFX)
      m_rMotorOutputVoltage = rightMaster.getFXSimCollection().getMotorOutputLeadVoltage();
    else if (rightMaster.getControllerType() == ControllerTypes.TalonSRX)
      m_rMotorOutputVoltage = rightMaster.getSRXSimCollection().getMotorOutputLeadVoltage();

    m_driveSim.setInputs(m_lMotorOutputVoltage, -m_rMotorOutputVoltage);
    m_driveSim.update(0.02);

    m_lPosition = m_driveConvertor.distanceMetersToNativeUnits(m_driveSim.getLeftPositionMeters());
    m_rPosition = m_driveConvertor.distanceMetersToNativeUnits(m_driveSim.getRightPositionMeters());
    m_lVelocity =
        m_driveConvertor.velocityToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rVelocity =
        m_driveConvertor.velocityToNativeUnits(m_driveSim.getRightVelocityMetersPerSecond());
    gyro.setRawHeadingDegrees(m_driveSim.getHeading().getDegrees());

    // Update all of our sensors.
    if (leftMaster.getControllerType() == ControllerTypes.TalonFX) {
      leftMaster.getFXSimCollection().setIntegratedSensorRawPosition(m_lPosition);
      leftMaster.getFXSimCollection().setIntegratedSensorVelocity(m_lVelocity);
      leftMaster.getFXSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
    } else if (leftMaster.getControllerType() == ControllerTypes.TalonSRX) {
      leftMaster.getSRXSimCollection().setQuadratureRawPosition(m_lPosition);
      leftMaster.getSRXSimCollection().setQuadratureVelocity(m_rVelocity);
      leftMaster.getFXSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
    }
    if (rightMaster.getControllerType() == ControllerTypes.TalonFX) {
      rightMaster.getFXSimCollection().setIntegratedSensorRawPosition(-m_rPosition);
      rightMaster.getFXSimCollection().setIntegratedSensorVelocity(m_rVelocity);
      rightMaster.getFXSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
    } else if (rightMaster.getControllerType() == ControllerTypes.TalonSRX) {
      rightMaster.getSRXSimCollection().setQuadratureRawPosition(m_lPosition);
      rightMaster.getSRXSimCollection().setQuadratureVelocity(m_rVelocity);
      rightMaster.getFXSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
    }
  }
}
