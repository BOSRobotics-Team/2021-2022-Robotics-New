// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Convertor;
import frc.robot.Gains;
import frc.robot.wrappers.SmartMotor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Intake extends SubsystemBase {
    private final WPI_TalonSRX _intakeController = new WPI_TalonSRX(5);
    private final WPI_TalonSRX _liftController = new WPI_TalonSRX(6);

    private boolean _isResetLift = false;
    private boolean _isLifting = false;
    private boolean _isFwdLimitSwitchTest = false;
    private boolean _isRevLimitSwitchTest = false;
    private double _targetPos = 0;
    private double kResetLiftSpeed = -0.2;
    private double kLiftGearRatio = 12.0;
    private double kLiftWheelRadius = 1.0;

    private boolean _isIntake = false;
    private double _targetVel = 0.0;

    private final int kSlotLift = 0;
    private final int kTimeoutMs = SmartMotor.kTimeoutMs;
    private final int kPIDLoopIdx = SmartMotor.PID_PRIMARY;
    private final Convertor _liftConvertor = new Convertor(4096, kLiftGearRatio, kLiftWheelRadius);

	public static final Gains kDefaultGains_Velocity = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
    public static final Gains kDefaultGains_Distanc = new Gains( 0.1, 0.0, 0.0, 0.0, 100, 0.80 );

    public Intake() {
        this.configController(_intakeController, kDefaultGains_Velocity);
        _intakeController.setNeutralMode(NeutralMode.Coast);
        
        this.configController(_liftController, kDefaultGains_Distanc);
        _liftController.setNeutralMode(NeutralMode.Brake);
    }

    public void configController(WPI_TalonSRX _controller, Gains gain) {
        _controller.configFactoryDefault();
        _controller.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0); 
        _controller.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        _controller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
        _controller.configNeutralDeadband(SmartMotor.kNeutralDeadband, kTimeoutMs);
        _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        _controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
        
        /* Set the peak and nominal outputs */
		_controller.configNominalOutputForward(0, kTimeoutMs);
		_controller.configNominalOutputReverse(0, kTimeoutMs);
		_controller.configPeakOutputForward(1, kTimeoutMs);
        _controller.configPeakOutputReverse(-1, kTimeoutMs);
        
		/* Set Motion Magic gains in slot0 - see documentation */
		_controller.selectProfileSlot(kSlotLift, kPIDLoopIdx);
        _controller.config_kF(kSlotLift, gain.kF, kTimeoutMs);
		_controller.config_kP(kSlotLift, gain.kP, kTimeoutMs);
		_controller.config_kI(kSlotLift, gain.kI, kTimeoutMs);
		_controller.config_kD(kSlotLift, gain.kD, kTimeoutMs);

   		/* Set acceleration and vcruise velocity - see documentation */
        _controller.configMotionCruiseVelocity(3000, kTimeoutMs);
        _controller.configMotionAcceleration(3000, kTimeoutMs);

        _controller.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
        if (_isResetLift) {
            if (isLiftRevLimitSwitchClosed()) {
                _isResetLift = false;
                _isLifting = false;
                _liftController.set(0.0);
                _liftController.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
            }
        }
        if (_isLifting) {
            System.out.println("isLifting - current pos = " + _liftController.getSelectedSensorPosition(kPIDLoopIdx));
            _liftController.set(ControlMode.MotionMagic, _targetPos);

            if (Math.abs(_liftController.getSelectedSensorPosition(kPIDLoopIdx) - _targetPos) < 0.01) {
                _isLifting = false;
                System.out.println("isLifting - done");
            } else if (isLiftFwdLimitSwitchClosed()) {
                _isLifting = false;
                System.out.println("isLifting - limit exceeded");
            }
        }
        if (_isIntake) {
            System.out.println("isIntake - current vel = " + _intakeController.getSelectedSensorVelocity(kPIDLoopIdx));
            _intakeController.set(ControlMode.Velocity, _targetVel);
        }
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void logPeriodic() {
        // _intakeController.logPeriodic();
     }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void runIntake(double speed) {
        _targetVel = _liftConvertor.velocityToNativeUnits(speed);
        _isIntake = true;

        System.out.println("isIntake - target (meters/sec) = " + speed);
    }
    public void stopIntake() {
        _isIntake = false;
        _intakeController.set(0.0);
        System.out.println("isIntake - stopped ");
    }

    public void runLift(double height) {
        _targetPos = _liftConvertor.distanceMetersToNativeUnits(height);
        _isLifting = true;

        System.out.println("isLifting - target (meters) = " + height);
    }
    public boolean isLifting() { return _isLifting; }
    public void setLift(double speed) {
        if (!_isLifting && !_isResetLift) {
            _liftController.set(speed);
        }
    }

    public void resetLift() {
        _isResetLift = true;
        _liftController.set(kResetLiftSpeed);
    }
    public boolean isResetting() { return _isResetLift; }
    public boolean isLiftFwdLimitSwitchClosed() { return (_liftController.isFwdLimitSwitchClosed() == 1) || _isFwdLimitSwitchTest; }
    public boolean isLiftRevLimitSwitchClosed() { return (_liftController.isRevLimitSwitchClosed() == 1) || _isRevLimitSwitchTest; }

    public void tripFwdLimitSwitches_test(boolean trip) {
        _isFwdLimitSwitchTest = trip;
    } 
    public void tripRevLimitSwitches_test(boolean trip) {
        System.out.println("tripRevLimitSwitches_test -" + trip);
        _isRevLimitSwitchTest = trip;
    } 
}

