package frc.robot.sim;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.Convertor;

public class ClimberSim {
  private final WPI_TalonFX _leftClimberController;
  private final WPI_TalonFX _rightClimberController;
  private final WPI_TalonFX _leftPivotLinkController;
  private final WPI_TalonFX _rightPivotLinkController;

  private final TalonFXSimCollection _leftClimberSim;
  private final TalonFXSimCollection _rightClimberSim;
  private final TalonFXSimCollection _leftPivotSim;
  private final TalonFXSimCollection _rightPivotSim;

  // Simulation classes help us simulate what's going on, including gravity.
  private final Convertor m_elevatorConvertor;
  private final double m_carriageMass;
  private final double m_minElevatorHeight;
  private final double m_lMaxElevatorHeight;
  private final double m_rMaxElevatorHeight;
  private final ElevatorSim m_lElevatorSim;
  private final ElevatorSim m_rElevatorSim;

  private final Convertor m_armConvertor;
  private final double m_armMass;
  private final double m_armLength;
  private final double m_lMinArmAngle;
  private final double m_lMaxArmAngle;
  private final double m_rMinArmAngle;
  private final double m_rMaxArmAngle;
  private final SingleJointedArmSim m_lPivotSim;
  private final SingleJointedArmSim m_rPivotSim;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d;
  private final MechanismRoot2d m_armLPivot;
  private final MechanismRoot2d m_armRPivot;
  private final MechanismRoot2d m_armLClimber;
  private final MechanismRoot2d m_armRClimber;
  private final MechanismLigament2d m_armL;
  private final MechanismLigament2d m_armR;
  private final MechanismLigament2d m_climbLTower;
  private final MechanismLigament2d m_climbRTower;
  private final MechanismLigament2d m_climbL;
  private final MechanismLigament2d m_climbR;

  public ClimberSim(
      WPI_TalonFX lClimber,
      WPI_TalonFX rClimber,
      Convertor climbConvertor,
      double climbMass,
      double lClimbHt,
      double rClimbHt,
      WPI_TalonFX lPivot,
      WPI_TalonFX rPivot,
      Convertor pivotConvertor,
      double armMass,
      double armLen,
      double lMinAngle,
      double lMaxAngle,
      double rMinAngle,
      double rMaxAngle) {
    _leftClimberController = lClimber;
    _rightClimberController = rClimber;
    _leftClimberSim = _leftClimberController.getSimCollection();
    _rightClimberSim = _rightClimberController.getSimCollection();

    m_elevatorConvertor = climbConvertor;
    m_carriageMass = climbMass;
    m_minElevatorHeight = 0;
    m_lMaxElevatorHeight = lClimbHt;
    m_rMaxElevatorHeight = rClimbHt;

    m_lElevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            m_elevatorConvertor.gearRatios.kGearRatio * m_elevatorConvertor.gearRatios.kPulleyRatio,
            m_carriageMass,
            Units.inchesToMeters(m_elevatorConvertor.gearRatios.kWheelRadiusInches),
            m_minElevatorHeight,
            m_lMaxElevatorHeight,
            null // VecBuilder.fill(0.01)
            );
    m_rElevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            m_elevatorConvertor.gearRatios.kGearRatio * m_elevatorConvertor.gearRatios.kPulleyRatio,
            m_carriageMass,
            Units.inchesToMeters(m_elevatorConvertor.gearRatios.kWheelRadiusInches),
            m_minElevatorHeight,
            m_rMaxElevatorHeight,
            null // VecBuilder.fill(0.01)
            );

    _leftPivotLinkController = lPivot;
    _rightPivotLinkController = rPivot;

    _leftPivotSim = _leftPivotLinkController.getSimCollection();
    _rightPivotSim = _rightPivotLinkController.getSimCollection();

    m_armConvertor = pivotConvertor;
    m_armMass = armMass;
    m_armLength = armLen;
    m_lMinArmAngle = Units.degreesToRadians(lMinAngle);
    m_lMaxArmAngle = Units.degreesToRadians(lMaxAngle);
    m_lPivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            m_armConvertor.gearRatios.kGearRatio * m_armConvertor.gearRatios.kPulleyRatio,
            SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
            m_armLength,
            m_lMinArmAngle,
            m_lMaxArmAngle,
            m_armMass,
            false,
            null // VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1
            );
    m_rMinArmAngle = Units.degreesToRadians(rMinAngle);
    m_rMaxArmAngle = Units.degreesToRadians(rMaxAngle);
    m_rPivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            m_armConvertor.gearRatios.kGearRatio * m_armConvertor.gearRatios.kPulleyRatio,
            SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
            m_armLength,
            m_rMinArmAngle,
            m_rMaxArmAngle,
            m_armMass,
            false,
            null // VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1
            );

    m_mech2d = new Mechanism2d(90, 60);
    m_armLPivot = m_mech2d.getRoot("ArmLPivot", 20, 0);
    m_armRPivot = m_mech2d.getRoot("ArmRPivot", 60, 0);
    m_armLClimber = m_mech2d.getRoot("ArmLClimber", 20, 0);
    m_armRClimber = m_mech2d.getRoot("ArmRClimber", 60, 0);
    m_armL =
        m_armLPivot.append(
            new MechanismLigament2d(
                "ArmL", Units.metersToInches(m_armLength), 0, 6, new Color8Bit(Color.kYellow)));
    m_armR =
        m_armRPivot.append(
            new MechanismLigament2d(
                "ArmR", Units.metersToInches(m_armLength), 0, 6, new Color8Bit(Color.kYellow)));
    m_climbLTower =
        m_armLClimber.append(
            new MechanismLigament2d("TowerL", 30, 90, 8, new Color8Bit(Color.kCadetBlue)));
    m_climbRTower =
        m_armRClimber.append(
            new MechanismLigament2d("TowerR", 30, 90, 8, new Color8Bit(Color.kCadetBlue)));
    m_climbL =
        m_climbLTower.append(
            new MechanismLigament2d(
                "ClimberL",
                Units.metersToInches(m_lMaxElevatorHeight),
                0,
                6,
                new Color8Bit(Color.kAqua)));
    m_climbR =
        m_climbRTower.append(
            new MechanismLigament2d(
                "ClimberR",
                Units.metersToInches(m_rMaxElevatorHeight),
                0,
                6,
                new Color8Bit(Color.kAqua)));

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("ClimberSim", m_mech2d);
  }

  public void run() {
    // This method will be called once per scheduler run during simulation
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_lElevatorSim.setInput(_leftClimberSim.getMotorOutputLeadVoltage());
    m_rElevatorSim.setInput(-_rightClimberSim.getMotorOutputLeadVoltage());
    m_lPivotSim.setInput(_leftPivotSim.getMotorOutputLeadVoltage());
    m_rPivotSim.setInput(-_rightPivotSim.getMotorOutputLeadVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_lElevatorSim.update(0.020);
    m_rElevatorSim.update(0.020);
    m_lPivotSim.update(0.020);
    m_rPivotSim.update(0.020);

    // Set our simulated encoder's readings and simulated battery voltage
    _leftClimberSim.setIntegratedSensorRawPosition(
        m_elevatorConvertor.distanceMetersToNativeUnits(m_lElevatorSim.getPositionMeters()));
    _leftClimberSim.setIntegratedSensorVelocity(
        m_elevatorConvertor.velocityToNativeUnits(m_lElevatorSim.getVelocityMetersPerSecond()));
    _leftClimberSim.setLimitRev(m_lElevatorSim.getPositionMeters() <= 0.0);

    _rightClimberSim.setIntegratedSensorRawPosition(
        -m_elevatorConvertor.distanceMetersToNativeUnits(m_rElevatorSim.getPositionMeters()));
    _rightClimberSim.setIntegratedSensorVelocity(
        m_elevatorConvertor.velocityToNativeUnits(m_rElevatorSim.getVelocityMetersPerSecond()));
    _rightClimberSim.setLimitRev(m_rElevatorSim.getPositionMeters() <= 0.0);

    _leftPivotSim.setIntegratedSensorRawPosition(
        m_armConvertor.distanceMetersToNativeUnits(
            Units.radiansToDegrees(m_lPivotSim.getAngleRads() - m_lMaxArmAngle)));
    _leftPivotSim.setIntegratedSensorVelocity(
        m_armConvertor.velocityToNativeUnits(m_lPivotSim.getVelocityRadPerSec()));
    _leftPivotSim.setLimitFwd(m_lPivotSim.getAngleRads() >= m_lMaxArmAngle);

    _rightPivotSim.setIntegratedSensorRawPosition(
        -m_armConvertor.distanceMetersToNativeUnits(
            Units.radiansToDegrees(m_rPivotSim.getAngleRads() - m_rMaxArmAngle)));
    _rightPivotSim.setIntegratedSensorVelocity(
        m_armConvertor.velocityToNativeUnits(m_rPivotSim.getVelocityRadPerSec()));
    _rightPivotSim.setLimitFwd(m_rPivotSim.getAngleRads() >= m_rMaxArmAngle);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_lElevatorSim.getCurrentDrawAmps()
                + m_rElevatorSim.getCurrentDrawAmps()
                + m_lPivotSim.getCurrentDrawAmps()
                + m_rPivotSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_armL.setAngle(Units.radiansToDegrees(m_lPivotSim.getAngleRads()));
    m_armR.setAngle(Units.radiansToDegrees(m_rPivotSim.getAngleRads()));
    m_climbL.setLength(Units.metersToInches(m_lElevatorSim.getPositionMeters()));
    m_climbR.setLength(Units.metersToInches(m_rElevatorSim.getPositionMeters()));
  }
}
