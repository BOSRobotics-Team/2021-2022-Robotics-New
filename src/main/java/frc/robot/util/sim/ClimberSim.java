package frc.robot.util.sim;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.Convertor;
import frc.robot.util.SmartMotor;

public class ClimberSim {
  private final SmartMotor m_leftClimberController;
  private final SmartMotor m_rightClimberController;
  private final SmartMotor m_leftPivotLinkController;
  private final SmartMotor m_rightPivotLinkController;

  private final TalonFXSimCollection m_leftClimberSim;
  private final TalonFXSimCollection m_rightClimberSim;
  private final TalonFXSimCollection m_leftPivotSim;
  private final TalonFXSimCollection m_rightPivotSim;

  // Simulation classes help us simulate what's going on, including gravity.
  private final double m_carriageMass;

  private final Convertor m_lElevatorConvertor;
  private final double m_lMinElevatorHeight;
  private final double m_lMaxElevatorHeight;
  private final ElevatorSim m_lElevatorSim;

  private final Convertor m_rElevatorConvertor;
  private final double m_rMinElevatorHeight;
  private final double m_rMaxElevatorHeight;
  private final ElevatorSim m_rElevatorSim;

  private final double m_armMass;
  private final double m_armLength;

  private final Convertor m_lArmConvertor;
  private final double m_lMinArmAngle;
  private final double m_lMaxArmAngle;
  private final SingleJointedArmSim m_lPivotSim;

  private final Convertor m_rArmConvertor;
  private final double m_rMinArmAngle;
  private final double m_rMaxArmAngle;
  private final SingleJointedArmSim m_rPivotSim;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d;
  private final MechanismRoot2d m_armLPivot;
  private final MechanismRoot2d m_armLClimber;
  private final MechanismLigament2d m_armL;
  private final MechanismLigament2d m_climbLTower;
  private final MechanismLigament2d m_climbL;

  private final MechanismRoot2d m_armRPivot;
  private final MechanismRoot2d m_armRClimber;
  private final MechanismLigament2d m_armR;
  private final MechanismLigament2d m_climbRTower;
  private final MechanismLigament2d m_climbR;

  public ClimberSim(
      SmartMotor lClimber,
      SmartMotor rClimber,
      double climbMass,
      double lClimbMinHt,
      double rClimbMinHt,
      double lClimbMaxHt,
      double rClimbMaxHt,
      SmartMotor lPivot,
      SmartMotor rPivot,
      double armMass,
      double armLen,
      double lMinAngle,
      double lMaxAngle,
      double rMinAngle,
      double rMaxAngle) {

    m_carriageMass = climbMass;

    m_leftClimberController = lClimber;
    m_leftClimberSim = m_leftClimberController.getFXSimCollection();
    m_lElevatorConvertor = m_leftClimberController.getConvertor();
    m_lMinElevatorHeight = lClimbMinHt;
    m_lMaxElevatorHeight = lClimbMaxHt;

    m_lElevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            m_lElevatorConvertor.gearRatios.kGearRatio
                * m_lElevatorConvertor.gearRatios.kPulleyRatio,
            m_carriageMass,
            Units.inchesToMeters(m_lElevatorConvertor.gearRatios.kWheelRadiusInches),
            m_lMinElevatorHeight,
            m_lMaxElevatorHeight,
            null // VecBuilder.fill(0.01)
            );

    m_rightClimberController = rClimber;
    m_rightClimberSim = m_rightClimberController.getFXSimCollection();
    m_rElevatorConvertor = m_rightClimberController.getConvertor();
    m_rMinElevatorHeight = rClimbMinHt;
    m_rMaxElevatorHeight = rClimbMaxHt;

    m_rElevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            m_rElevatorConvertor.gearRatios.kGearRatio
                * m_rElevatorConvertor.gearRatios.kPulleyRatio,
            m_carriageMass,
            Units.inchesToMeters(m_rElevatorConvertor.gearRatios.kWheelRadiusInches),
            m_rMinElevatorHeight,
            m_rMaxElevatorHeight,
            null // VecBuilder.fill(0.01)
            );

    m_armMass = armMass;
    m_armLength = armLen;

    m_leftPivotLinkController = lPivot;
    m_leftPivotSim = m_leftPivotLinkController.getFXSimCollection();
    m_lArmConvertor = m_leftPivotLinkController.getConvertor();
    m_lMinArmAngle = Units.degreesToRadians(lMinAngle);
    m_lMaxArmAngle = Units.degreesToRadians(lMaxAngle);
    m_lPivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            m_lArmConvertor.gearRatios.kGearRatio * m_lArmConvertor.gearRatios.kPulleyRatio,
            SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
            m_armLength,
            m_lMinArmAngle,
            m_lMaxArmAngle,
            m_armMass,
            false,
            null // VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1
            );

    m_rightPivotLinkController = rPivot;
    m_rightPivotSim = m_rightPivotLinkController.getFXSimCollection();
    m_rArmConvertor = m_rightPivotLinkController.getConvertor();
    m_rMinArmAngle = Units.degreesToRadians(rMinAngle);
    m_rMaxArmAngle = Units.degreesToRadians(rMaxAngle);
    m_rPivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            m_rArmConvertor.gearRatios.kGearRatio * m_rArmConvertor.gearRatios.kPulleyRatio,
            SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
            m_armLength,
            m_rMinArmAngle,
            m_rMaxArmAngle,
            m_armMass,
            false,
            null // VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1
            );

    m_mech2d = new Mechanism2d(90, 60);
    m_armLClimber = m_mech2d.getRoot("ArmLClimber", 20, 0);
    m_climbLTower =
        m_armLClimber.append(
            new MechanismLigament2d("TowerL", 30, 90, 8, new Color8Bit(Color.kCadetBlue)));
    m_climbL =
        m_climbLTower.append(
            new MechanismLigament2d(
                "ClimberL",
                Units.metersToInches(m_lMaxElevatorHeight),
                0,
                6,
                new Color8Bit(Color.kAqua)));

    m_armRClimber = m_mech2d.getRoot("ArmRClimber", 60, 0);
    m_climbRTower =
        m_armRClimber.append(
            new MechanismLigament2d("TowerR", 30, 90, 8, new Color8Bit(Color.kCadetBlue)));
    m_climbR =
        m_climbRTower.append(
            new MechanismLigament2d(
                "ClimberR",
                Units.metersToInches(m_rMaxElevatorHeight),
                0,
                6,
                new Color8Bit(Color.kAqua)));
    m_armLPivot = m_mech2d.getRoot("ArmLPivot", 20, 0);
    m_armL =
        m_armLPivot.append(
            new MechanismLigament2d(
                "ArmL", Units.metersToInches(m_armLength), 0, 6, new Color8Bit(Color.kYellow)));
    m_armRPivot = m_mech2d.getRoot("ArmRPivot", 60, 0);
    m_armR =
        m_armRPivot.append(
            new MechanismLigament2d(
                "ArmR", Units.metersToInches(m_armLength), 0, 6, new Color8Bit(Color.kYellow)));

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("ClimberSim", m_mech2d);
  }

  public void run() {
    // This method will be called once per scheduler run during simulation
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_lElevatorSim.setInput(m_leftClimberSim.getMotorOutputLeadVoltage());
    m_rElevatorSim.setInput(-m_rightClimberSim.getMotorOutputLeadVoltage());
    m_lPivotSim.setInput(m_leftPivotSim.getMotorOutputLeadVoltage());
    m_rPivotSim.setInput(-m_rightPivotSim.getMotorOutputLeadVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_lElevatorSim.update(0.020);
    m_rElevatorSim.update(0.020);
    m_lPivotSim.update(0.020);
    m_rPivotSim.update(0.020);

    // Set our simulated encoder's readings and simulated battery voltage
    m_leftClimberSim.setIntegratedSensorRawPosition(
        m_lElevatorConvertor.distanceMetersToNativeUnits(m_lElevatorSim.getPositionMeters()));
    m_leftClimberSim.setIntegratedSensorVelocity(
        m_lElevatorConvertor.velocityToNativeUnits(m_lElevatorSim.getVelocityMetersPerSecond()));
    m_leftClimberSim.setLimitRev(m_lElevatorSim.getPositionMeters() <= 0.0);

    m_rightClimberSim.setIntegratedSensorRawPosition(
        -m_rElevatorConvertor.distanceMetersToNativeUnits(m_rElevatorSim.getPositionMeters()));
    m_rightClimberSim.setIntegratedSensorVelocity(
        m_rElevatorConvertor.velocityToNativeUnits(m_rElevatorSim.getVelocityMetersPerSecond()));
    m_rightClimberSim.setLimitRev(m_rElevatorSim.getPositionMeters() <= 0.0);

    m_leftPivotSim.setIntegratedSensorRawPosition(
        m_lArmConvertor.distanceMetersToNativeUnits(
            Units.radiansToDegrees(m_lPivotSim.getAngleRads() - m_lMaxArmAngle)));
    m_leftPivotSim.setIntegratedSensorVelocity(
        m_lArmConvertor.velocityToNativeUnits(m_lPivotSim.getVelocityRadPerSec()));
    m_leftPivotSim.setLimitFwd(m_lPivotSim.getAngleRads() >= m_lMaxArmAngle);

    m_rightPivotSim.setIntegratedSensorRawPosition(
        -m_rArmConvertor.distanceMetersToNativeUnits(
            Units.radiansToDegrees(m_rPivotSim.getAngleRads() - m_rMaxArmAngle)));
    m_rightPivotSim.setIntegratedSensorVelocity(
        m_rArmConvertor.velocityToNativeUnits(m_rPivotSim.getVelocityRadPerSec()));
    m_rightPivotSim.setLimitFwd(m_rPivotSim.getAngleRads() >= m_rMaxArmAngle);

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
