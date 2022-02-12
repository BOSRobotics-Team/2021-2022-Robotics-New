// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.*;

public class Climber extends SubsystemBase {
  private final WPI_TalonFX _leftclimberController = new WPI_TalonFX(Constants.kID_LClimber);
  private final WPI_TalonFX _rightclimberController = new WPI_TalonFX(Constants.kID_RClimber);
  private final WPI_TalonFX _leftPivotLinkController = new WPI_TalonFX(Constants.kID_LPivot);
  private final WPI_TalonFX _rightPivotLinkController = new WPI_TalonFX(Constants.kID_RPivot);

  private final SmartMotorController smartClimberController =
      new SmartMotorController(_rightclimberController, _leftclimberController);
  private final SmartMotorController smartPivotLinkController =
      new SmartMotorController(_rightPivotLinkController, _leftPivotLinkController);

  private final GearRatios kGearRatio_Climber = new GearRatios(12.0, 1.0, 1.0);
  public final Gains kGains_Climber = new Gains(0.5, 0.0, 0.0, 0.5, 0, 1.0);
  private boolean _isClimbing = false;
  private boolean _isResetClimber = false;
  private double _lastLClimberHeight = 0.0;
  private double _lastRClimberHeight = 0.0;
  private double _targetClimberHeight = 0;
  private double kResetClimberSpeed = -0.05;
  private double _climberMaxHeight = 1.5;
  // private double _climberFeedFwd = 0.1;

  private final GearRatios kGearRatio_PivotLink = new GearRatios(100.0, 1.0, 2.0);
  public final Gains kGains_PivotLink = new Gains(0.2, 0.0, 0.0, 0.2, 0, 0.4);
  private boolean _isPivoting = false;
  private boolean _isResetPivoting = false;
  private double _lastLPivotDistance = 0.0;
  private double _lastRPivotDistance = 0.0;
  private double _targetPivotDistance = 0;
  private double kResetPivotSpeed = -0.05;
  private double _pivotMaxDistance = 0.08;

  // Simulation classes help us simulate what's going on, including gravity.
  private static final double m_carriageMass = 0.1; // 10.0; // Kilograms
  private static final double m_drumRadius = Units.inchesToMeters(2);
  private static final double m_minElevatorHeight = Units.inchesToMeters(0);
  private static final double m_maxElevatorHeight = Units.inchesToMeters(30);
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getFalcon500(1),
          kGearRatio_Climber.kGearRatio,
          m_carriageMass,
          m_drumRadius,
          m_minElevatorHeight,
          m_maxElevatorHeight,
          null // VecBuilder.fill(0.01)
          );

  private static final double m_armMass = 0.1; // 3.0; // Kilograms
  private static final double m_armLength = Units.inchesToMeters(30);
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          kGearRatio_PivotLink.kGearRatio * 2,
          SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
          m_armLength,
          Units.degreesToRadians(40),
          Units.degreesToRadians(140),
          m_armMass,
          true,
          null // VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1
          // tick
          );

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 0);
  private final MechanismRoot2d m_armClimber = m_mech2d.getRoot("ArmClimber", 30, 0);
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              Units.metersToInches(m_armLength),
              180 - Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));
  private final MechanismLigament2d m_climbTower =
      m_armClimber.append(
          new MechanismLigament2d("HookTower", 25, 90, 8, new Color8Bit(Color.kAqua)));
  private final MechanismLigament2d m_climb =
      m_climbTower.append(
          new MechanismLigament2d(
              "Hook",
              Units.metersToInches(m_elevatorSim.getPositionMeters()),
              0,
              6,
              new Color8Bit(Color.kAqua)));

  public Climber() {
    Preferences.initDouble("ClimberMaxHeight", 1.1);
    Preferences.initDouble("PivotMaxDistance", 0.06);
    // Preferences.initDouble("ClimberFeedFwd", 0.5);

    _climberMaxHeight = Preferences.getDouble("ClimberMaxHeight", 1.1);
    _pivotMaxDistance = Preferences.getDouble("PivotMaxDistance", 0.06);
    // _climberFeedFwd = Preferences.getDouble("ClimberFeedFwd", 0.1);

    _leftclimberController.setInverted(InvertType.None);
    _rightclimberController.setInverted(InvertType.InvertMotorOutput);

    smartClimberController.initController();
    smartClimberController.configureRatios(kGearRatio_Climber);
    smartClimberController.enableBrakes(true);
    smartClimberController.setDistanceAndTurnConfigs(kGains_Climber, kGains_Climber);
    smartClimberController.resetPosition();

    _leftPivotLinkController.setInverted(InvertType.None);
    _rightPivotLinkController.setInverted(InvertType.InvertMotorOutput);

    smartPivotLinkController.initController();
    smartPivotLinkController.configureRatios(kGearRatio_PivotLink);
    smartPivotLinkController.enableBrakes(true);
    smartPivotLinkController.setDistanceAndTurnConfigs(kGains_PivotLink, kGains_PivotLink);
    smartPivotLinkController.resetPosition();

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if (_isResetClimber) {
      double LPos = smartClimberController.getLeftPosition();
      double RPos = smartClimberController.getRightPosition();
      boolean isLDone = (Math.abs(LPos - _lastLClimberHeight) < 5.0);
      boolean isRDone = (Math.abs(RPos - _lastRClimberHeight) < 5.0);

      if (isLDone) _leftclimberController.set(0.0);
      if (isRDone) _rightclimberController.set(0.0);
      if (isLDone && isRDone) {
        _isResetClimber = false;
        _isClimbing = false;
        smartClimberController.resetPosition();
      }
      _lastLClimberHeight = LPos;
      _lastRClimberHeight = RPos;
    }
    if (_isResetPivoting) {
      double LPos = smartPivotLinkController.getLeftPosition();
      double RPos = smartPivotLinkController.getRightPosition();
      boolean isLDone = (Math.abs(LPos - _lastLPivotDistance) < 5.0);
      boolean isRDone = (Math.abs(RPos - _lastRPivotDistance) < 5.0);

      if (isLDone) _leftPivotLinkController.set(0.0);
      if (isRDone) _rightPivotLinkController.set(0.0);
      if (isLDone && isRDone) {
        _isResetPivoting = false;
        _isPivoting = false;
        smartPivotLinkController.resetPosition();
      }
      _lastLPivotDistance = LPos;
      _lastRPivotDistance = RPos;
    }
    if (_isClimbing) {
      System.out.println(
          "isClimbing - current height = "
              + smartClimberController.getDistance()
              + " pos = "
              + smartClimberController.getPosition());
      if (smartClimberController.isTargetFinished()) {
        _isClimbing = false;
        System.out.println("isClimbing - done");
      }
    }
    if (_isPivoting) {
      System.out.println(
          "isPivoting - current distance = "
              + smartPivotLinkController.getDistance()
              + smartPivotLinkController.getPosition());
      if (smartPivotLinkController.isTargetFinished()) {
        _isPivoting = false;
        System.out.println("isPivoting - done");
      }
    }
    smartClimberController.update();
    smartPivotLinkController.update();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(_leftclimberController.getSimCollection().getMotorOutputLeadVoltage());
    m_armSim.setInput(_leftPivotLinkController.getSimCollection().getMotorOutputLeadVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    // m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // m_encoderSim.setDistance(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_elevatorSim.getCurrentDrawAmps() + m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    m_climb.setLength(Units.metersToInches(m_elevatorSim.getPositionMeters()));
  }

  public void logPeriodic() {
    SmartDashboard.putNumber("targetHeight", _targetClimberHeight);
    SmartDashboard.putNumber("targetPivot", _targetPivotDistance);
    SmartDashboard.putNumber("ClimberPos", smartClimberController.getPosition());
    SmartDashboard.putNumber("ClimberLPos", smartClimberController.getLeftPosition());
    SmartDashboard.putNumber("ClimberRPos", smartClimberController.getRightPosition());
    SmartDashboard.putNumber("PivotPos", smartPivotLinkController.getPosition());
    SmartDashboard.putNumber("PivotLPos", smartPivotLinkController.getLeftPosition());
    SmartDashboard.putNumber("PivotRPos", smartPivotLinkController.getRightPosition());
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void setClimberHeight(double pctHeight) {
    _targetClimberHeight = MathUtil.clamp(pctHeight, 0.0, 1.0) * _climberMaxHeight;
    smartClimberController.setTarget(_targetClimberHeight, 0); // , _climberFeedFwd);
    _isClimbing = true;
  }

  public double getClimberHeight() {
    return smartClimberController.getDistance() / _climberMaxHeight;
  }

  public boolean isClimbing() {
    return _isClimbing;
  }

  public void setClimberSpeed(double speed) {
    if (!_isResetClimber) {
      smartClimberController.set(speed);
      _isClimbing = false;
    }
  }

  public void resetClimber() {
    _isResetClimber = true;
    _isClimbing = false;
    _lastLClimberHeight = smartClimberController.getLeftPosition();
    _lastRClimberHeight = smartClimberController.getRightPosition();
    smartClimberController.set(kResetClimberSpeed, kResetClimberSpeed);
  }

  public void setPivotLinkDistance(double pctDistance) {
    _targetPivotDistance = MathUtil.clamp(pctDistance, 0.0, 1.0) * _pivotMaxDistance;
    smartPivotLinkController.setTarget(_targetPivotDistance, 0); // , _climberFeedFwd);
    _isPivoting = true;
  }

  public double getPivotLinkDistance() {
    return smartPivotLinkController.getDistance() / _pivotMaxDistance;
  }

  public boolean isPivoting() {
    return _isPivoting;
  }

  public void setPivotLinkSpeed(double speed) {
    if (!_isResetPivoting) {
      smartPivotLinkController.set(speed);
      _isPivoting = false;
    }
  }

  public void resetPivotLink() {
    _isPivoting = false;
    _isResetPivoting = true;
    _lastLPivotDistance = smartPivotLinkController.getLeftPosition();
    _lastRPivotDistance = smartPivotLinkController.getRightPosition();
    smartPivotLinkController.set(kResetPivotSpeed, kResetPivotSpeed);
  }

  public boolean isResetting() {
    return _isResetClimber || _isResetPivoting;
  }
}
