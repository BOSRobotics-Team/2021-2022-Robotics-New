package frc.robot.sim;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.*;
import frc.robot.util.*;
import frc.robot.wrappers.*;

public class DrivetrainSim {
  private final WPI_TalonFX leftMaster;
  private final WPI_TalonFX rightMaster;

  private final TalonFXSimCollection leftMasterSim;
  private final TalonFXSimCollection rightMasterSim;

  /** The NavX gyro */
  private final DriveGyro gyro;

  private final Convertor m_driveConvertor;
  private final double m_driveMass;
  private final double m_driveWidth;

  /* Simulation model of the drivetrain */
  private final DifferentialDrivetrainSim m_driveSim;

  public DrivetrainSim(
      WPI_TalonFX left,
      WPI_TalonFX right,
      DriveGyro gy,
      Convertor driveConvertor,
      double driveMass,
      double driveWidth) {
    leftMaster = left;
    rightMaster = right;

    leftMasterSim = leftMaster.getSimCollection();
    rightMasterSim = rightMaster.getSimCollection();

    gyro = gy;
    m_driveConvertor = driveConvertor;
    m_driveMass = driveMass;
    m_driveWidth = driveWidth;

    m_driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2), // 2 CIMS on each side of the drivetrain.
            driveConvertor.gearRatios.kGearRatio, // Standard AndyMark Gearing reduction.
            2.1, // MOI of 2.1 kg m^2 (from CAD model).
            m_driveMass, // 26.5, // Mass of the robot is 26.5 kg.
            Units.inchesToMeters(driveConvertor.gearRatios.kWheelRadiusInches), // 3" radius wheels.
            m_driveWidth, // Distance between wheels is _ meters.
            null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
            );
  }

  public void run() {
    m_driveSim.setInputs(
        leftMasterSim.getMotorOutputLeadVoltage(), -rightMasterSim.getMotorOutputLeadVoltage());

    m_driveSim.update(0.02);

    // Update all of our sensors.
    leftMasterSim.setIntegratedSensorRawPosition(
        m_driveConvertor.distanceMetersToNativeUnits(m_driveSim.getLeftPositionMeters()));
    rightMasterSim.setIntegratedSensorRawPosition(
        -m_driveConvertor.distanceMetersToNativeUnits(m_driveSim.getRightPositionMeters()));
    leftMasterSim.setIntegratedSensorVelocity(
        m_driveConvertor.velocityToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond()));
    rightMasterSim.setIntegratedSensorVelocity(
        m_driveConvertor.velocityToNativeUnits(m_driveSim.getRightVelocityMetersPerSecond()));

    gyro.setRawHeadingDegrees(m_driveSim.getHeading().getDegrees());

    // Update other inputs to Talons
    leftMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
  }
}
