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
  private final double m_maxElevatorHeight;
  private final ElevatorSim m_lElevatorSim;
  private final ElevatorSim m_rElevatorSim;

  private final Convertor m_armConvertor;
  private final double m_armMass;
  private final double m_armLength;
  private final double m_minArmAngle;
  private final double m_maxArmAngle;
  private final SingleJointedArmSim m_lPivotSim;
  private final SingleJointedArmSim m_rPivotSim;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d;
  private final MechanismRoot2d m_armPivot;
  private final MechanismRoot2d m_armClimber;
  private final MechanismLigament2d m_arm;
  private final MechanismLigament2d m_climbTower;
  private final MechanismLigament2d m_climb;

  public ClimberSim(
      WPI_TalonFX lClimber,
      WPI_TalonFX rClimber,
      Convertor climbConvertor,
      double climbMass,
      double climbHt,
      WPI_TalonFX lPivot,
      WPI_TalonFX rPivot,
      Convertor pivotConvertor,
      double armMass,
      double armLen,
      double minAngle,
      double maxAngle) {
    _leftClimberController = lClimber;
    _rightClimberController = rClimber;
    _leftClimberSim = _leftClimberController.getSimCollection();
    _rightClimberSim = _rightClimberController.getSimCollection();

    m_elevatorConvertor = climbConvertor;
    m_carriageMass = climbMass;
    m_minElevatorHeight = 0;
    m_maxElevatorHeight = climbHt;

    m_lElevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            climbConvertor.gearRatios.kGearRatio * climbConvertor.gearRatios.kPulleyRatio,
            m_carriageMass,
            Units.inchesToMeters(climbConvertor.gearRatios.kWheelRadiusInches),
            m_minElevatorHeight,
            m_maxElevatorHeight,
            null // VecBuilder.fill(0.01)
            );
    m_rElevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            climbConvertor.gearRatios.kGearRatio * climbConvertor.gearRatios.kPulleyRatio,
            m_carriageMass,
            Units.inchesToMeters(climbConvertor.gearRatios.kWheelRadiusInches),
            m_minElevatorHeight,
            m_maxElevatorHeight,
            null // VecBuilder.fill(0.01)
            );

    _leftPivotLinkController = lPivot;
    _rightPivotLinkController = rPivot;

    _leftPivotSim = _leftPivotLinkController.getSimCollection();
    _rightPivotSim = _rightPivotLinkController.getSimCollection();

    m_armConvertor = pivotConvertor;
    m_armMass = armMass;
    m_armLength = armLen;
    m_minArmAngle = Units.degreesToRadians(minAngle);
    m_maxArmAngle = Units.degreesToRadians(maxAngle);
    m_lPivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            m_armConvertor.gearRatios.kGearRatio * m_armConvertor.gearRatios.kPulleyRatio,
            SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
            m_armLength,
            m_minArmAngle,
            m_maxArmAngle,
            m_armMass,
            false,
            null // VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1
            );
    m_rPivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            m_armConvertor.gearRatios.kGearRatio * m_armConvertor.gearRatios.kPulleyRatio,
            SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
            m_armLength,
            m_minArmAngle,
            m_maxArmAngle,
            m_armMass,
            false,
            null // VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1
            );

    m_mech2d = new Mechanism2d(60, 60);
    m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 0);
    m_armClimber = m_mech2d.getRoot("ArmClimber", 30, 0);
    m_arm =
        m_armPivot.append(
            new MechanismLigament2d(
                "Arm", Units.metersToInches(m_armLength), 180, 6, new Color8Bit(Color.kYellow)));
    m_climbTower =
        m_armClimber.append(
            new MechanismLigament2d("HookTower", 25, 90, 8, new Color8Bit(Color.kAqua)));
    m_climb =
        m_climbTower.append(
            new MechanismLigament2d(
                "Hook",
                Units.metersToInches(m_maxElevatorHeight),
                0,
                6,
                new Color8Bit(Color.kAqua)));

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
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
    _rightClimberSim.setIntegratedSensorRawPosition(
        -m_elevatorConvertor.distanceMetersToNativeUnits(m_rElevatorSim.getPositionMeters()));
    _rightClimberSim.setIntegratedSensorVelocity(
        m_elevatorConvertor.velocityToNativeUnits(m_rElevatorSim.getVelocityMetersPerSecond()));

    _leftPivotSim.setIntegratedSensorRawPosition(
        m_armConvertor.distanceMetersToNativeUnits(
            Units.radiansToDegrees(m_lPivotSim.getAngleRads() - m_maxArmAngle)));
    _rightPivotSim.setIntegratedSensorRawPosition(
        -m_armConvertor.distanceMetersToNativeUnits(
            Units.radiansToDegrees(m_rPivotSim.getAngleRads() - m_maxArmAngle)));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_lElevatorSim.getCurrentDrawAmps()
                + m_rElevatorSim.getCurrentDrawAmps()
                + m_lPivotSim.getCurrentDrawAmps()
                + m_rPivotSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_lPivotSim.getAngleRads()));
    m_climb.setLength(Units.metersToInches(m_lElevatorSim.getPositionMeters()));
  }
}
