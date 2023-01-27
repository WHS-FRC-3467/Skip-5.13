// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private TalonFX m_lowerJoint = new WPI_TalonFX(CanConstants.LOWER_JOINT_MOTOR);
  private TalonFX m_upperJoint = new TalonFX(CanConstants.UPPER_JOINT_MOTOR);

  private DutyCycleEncoder m_upperEncoder = new DutyCycleEncoder(DIOConstants.UPPER_ENCODER_ARM);
  private DutyCycleEncoder m_lowerEncoder = new DutyCycleEncoder(DIOConstants.LOWER_ENCODER_ARM);

  
  public ArmSubsystem() {    
    //Config Duty Cycle Range for the encoders
    m_lowerEncoder.setDutyCycleRange(ArmConstants.DUTY_CYCLE_MIN, ArmConstants.DUTY_CYCLE_MAX);
    m_upperEncoder.setDutyCycleRange(ArmConstants.DUTY_CYCLE_MIN, ArmConstants.DUTY_CYCLE_MAX);

    //Default Motors
    m_lowerJoint.configFactoryDefault(ArmConstants.TIMEOUT);
    m_upperJoint.configFactoryDefault(ArmConstants.TIMEOUT);

    // m_lowerJoint.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, ArmConstants.TIMEOUT, ArmConstants.TIMEOUT);
		// m_lowerJoint.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, ArmConstants.TIMEOUT, ArmConstants.TIMEOUT);
    // m_upperJoint.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, ArmConstants.TIMEOUT, ArmConstants.TIMEOUT);
		// m_upperJoint.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, ArmConstants.TIMEOUT, ArmConstants.TIMEOUT);

    //Set Neutral Mode to Brake and NeutralDeadBand to prevent need for intentional stalling
    m_lowerJoint.setNeutralMode(NeutralMode.Brake);
    m_upperJoint.setNeutralMode(NeutralMode.Brake);

    m_lowerJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);
    m_upperJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);

    m_lowerJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, ArmConstants.TIMEOUT);
    m_upperJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, ArmConstants.TIMEOUT);

    m_upperJoint.setInverted(TalonFXInvertType.Clockwise);
    m_lowerJoint.setInverted(TalonFXInvertType.Clockwise);

    m_lowerJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    m_lowerJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
    m_lowerJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    m_lowerJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

    m_upperJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    m_upperJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
    m_upperJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    m_upperJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

    //PID configs
    m_upperJoint.config_kP(0, ArmConstants.GAINS_UPPER_JOINT.kP, ArmConstants.TIMEOUT);
    m_upperJoint.config_kI(0, ArmConstants.GAINS_UPPER_JOINT.kI, ArmConstants.TIMEOUT);
    m_upperJoint.config_kD(0, ArmConstants.GAINS_UPPER_JOINT.kD, ArmConstants.TIMEOUT);
    m_upperJoint.config_IntegralZone(0, ArmConstants.GAINS_UPPER_JOINT.kIzone, ArmConstants.TIMEOUT);
    m_upperJoint.configAllowableClosedloopError(0, ArmConstants.TOLERANCE_UPPER, ArmConstants.TIMEOUT);

    m_lowerJoint.config_kP(0, ArmConstants.GAINS_LOWER_JOINT.kP, ArmConstants.TIMEOUT);
    m_lowerJoint.config_kI(0, ArmConstants.GAINS_LOWER_JOINT.kI, ArmConstants.TIMEOUT);
    m_lowerJoint.config_kD(0, ArmConstants.GAINS_LOWER_JOINT.kD, ArmConstants.TIMEOUT);
    m_lowerJoint.config_IntegralZone(0, ArmConstants.GAINS_LOWER_JOINT.kIzone, ArmConstants.TIMEOUT);
    m_lowerJoint.configAllowableClosedloopError(0, ArmConstants.TOLERANCE_LOWER, ArmConstants.TIMEOUT);

    //Motion Magic configs
    m_lowerJoint.configMotionAcceleration(ArmConstants.MOTION_ACCELERATION_LOWER, ArmConstants.TIMEOUT);
    m_lowerJoint.configMotionCruiseVelocity(ArmConstants.MOTION_CRUISE_VELOCITY_LOWER, ArmConstants.TIMEOUT);
    m_lowerJoint.configMotionSCurveStrength(ArmConstants.CURVE_SMOOTHING_LOWER, ArmConstants.TIMEOUT);

    m_upperJoint.configMotionAcceleration(ArmConstants.MOTION_ACCELERATION_UPPER, ArmConstants.TIMEOUT);
    m_upperJoint.configMotionCruiseVelocity(ArmConstants.MOTION_CRUISE_VELOCITY_UPPER, ArmConstants.TIMEOUT);
    m_upperJoint.configMotionSCurveStrength(ArmConstants.CURVE_SMOOTHING_UPPER, ArmConstants.TIMEOUT);

    m_lowerJoint.configClosedloopRamp(ArmConstants.CLOSED_LOOP_RAMP_LOWER, ArmConstants.TIMEOUT);
    m_upperJoint.configClosedloopRamp(ArmConstants.CLOSED_LOOP_RAMP_UPPER, ArmConstants.TIMEOUT);


    m_lowerJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);
    m_upperJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);

    m_lowerJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);
    m_upperJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);

    m_upperJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_UPPER, ArmConstants.TIMEOUT);
    m_upperJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_UPPER, ArmConstants.TIMEOUT);
    m_lowerJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);
    m_lowerJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);

    if(Constants.tuningMode){
      // SmartDashboard.putNumber("Upper P", ArmConstants.GAINS_UPPER_JOINT.kP);
      // SmartDashboard.putNumber("Upper I", ArmConstants.GAINS_UPPER_JOINT.kI);
      // SmartDashboard.putNumber("Upper D", ArmConstants.GAINS_UPPER_JOINT.kD);
      // SmartDashboard.putNumber("Lower P", ArmConstants.GAINS_LOWER_JOINT.kP);
      // SmartDashboard.putNumber("Lower I", ArmConstants.GAINS_LOWER_JOINT.kI);
      // SmartDashboard.putNumber("Lower D", ArmConstants.GAINS_LOWER_JOINT.kD);
      // SmartDashboard.putNumber("Upper Setpoint", 0.0);
      // SmartDashboard.putNumber("Lower Setpoint", 0.0); 
    }
    else{
      SmartDashboard.clearPersistent("Upper P");
      SmartDashboard.clearPersistent("Upper I");
      SmartDashboard.clearPersistent("Upper D");
      SmartDashboard.clearPersistent("Lower P");
      SmartDashboard.clearPersistent("Lower I");
      SmartDashboard.clearPersistent("Lower D");
      SmartDashboard.clearPersistent("Upper Setpoint");
      SmartDashboard.clearPersistent("Lower Setpoint");
      SmartDashboard.clearPersistent("Lower Cruise");
      SmartDashboard.clearPersistent("Lower Accel");
      SmartDashboard.clearPersistent("Lower Curve");
      SmartDashboard.clearPersistent("Upper Cruise");
      SmartDashboard.clearPersistent("Upper Accel");
      SmartDashboard.clearPersistent("Upper Curve");
      SmartDashboard.clearPersistent("Upper CLR");
      SmartDashboard.clearPersistent("Lower CLR");
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_upperJoint.setSelectedSensorPosition(dutyCycleToCTREUnits(getUpperJointPos()), 0, ArmConstants.TIMEOUT);
    m_lowerJoint.setSelectedSensorPosition(dutyCycleToCTREUnits(getLowerJointPos()), 0, ArmConstants.TIMEOUT);

    if(Constants.tuningMode){
      // SmartDashboard.putNumber("Lower CTRE", dutyCycleToCTREUnits(getLowerJointPos()));
      // SmartDashboard.putNumber("Upper CTRE", dutyCycleToCTREUnits(getUpperJointPos()));
      SmartDashboard.putNumber("Lower Angle", dutyCycleToDegrees(getLowerJointPos()));
      SmartDashboard.putNumber("Upper Angle", dutyCycleToDegrees(getUpperJointPos()));

      // SmartDashboard.putNumber("Upper Sensor", m_upperJoint.getSelectedSensorPosition());
      // SmartDashboard.putNumber("Lower Sensor", m_lowerJoint.getSelectedSensorPosition());
      // SmartDashboard.putNumber("Upper Target", m_upperJoint.getClosedLoopTarget());
      // SmartDashboard.putNumber("Lower Target", m_lowerJoint.getClosedLoopTarget());
      // SmartDashboard.putNumber("Upper Error", m_upperJoint.getClosedLoopError());
      // SmartDashboard.putNumber("Lower Error", m_lowerJoint.getClosedLoopError());   
      SmartDashboard.putNumber("Upper Percent", m_upperJoint.getMotorOutputPercent());
      SmartDashboard.putNumber("Lower Percent", m_lowerJoint.getMotorOutputPercent());
      SmartDashboard.putNumber("Upper Abs", getUpperJointPos());
      SmartDashboard.putNumber("Lower Abs", getLowerJointPos());
      // SmartDashboard.putNumber("Upper Sensor Vel", m_upperJoint.getSe());
    }
    else{
      SmartDashboard.clearPersistent("Lower CTRE");
      SmartDashboard.clearPersistent("Upper CTRE");
      SmartDashboard.clearPersistent("Lower Angle");
      SmartDashboard.clearPersistent("Upper Angle");

      SmartDashboard.clearPersistent("Upper Sensor");
      SmartDashboard.clearPersistent("Lower Sensor");
      SmartDashboard.clearPersistent("Upper Target");
      SmartDashboard.clearPersistent("Lower Target");
      SmartDashboard.clearPersistent("Upper Error");
      SmartDashboard.clearPersistent("Lower Error");   
    }
  }

  public void setPercentOutputUpper(double speed){
    m_upperJoint.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setSimplePIDUpper(double position){
    System.out.println("running SimplePIDUpper");
    m_upperJoint.set(TalonFXControlMode.Position, position, DemandType.ArbitraryFeedForward, ArmConstants.GAINS_UPPER_JOINT.kF);
  }

  public void setMotionMagicUpper(double position){
    m_upperJoint.set(TalonFXControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ArmConstants.GAINS_UPPER_JOINT.kF);
  }

  public void setPercentOutputLower(double speed){
    m_lowerJoint.set(ControlMode.PercentOutput, speed * 0.1);
  }

  public void setSimplePIDLower(double position){
    m_lowerJoint.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, ArmConstants.GAINS_LOWER_JOINT.kF);
  }

  public void setMotionMagicLower(double position){
    m_lowerJoint.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ArmConstants.GAINS_LOWER_JOINT.kF);
  }

  public void holdPositionUpper(){
    m_lowerJoint.neutralOutput();;
  }
  public void holdPositionLower(){
    m_lowerJoint.neutralOutput();
  }

  public double getLowerJointPos(){
    return m_lowerEncoder.getAbsolutePosition();
  }

  public double getUpperJointPos(){
    return m_upperEncoder.getAbsolutePosition();
  }
    
  public double dutyCycleToCTREUnits(double dutyCyclePos){
    //4096 units per rotation = raw sensor units for Pulse width encoder
    return dutyCyclePos * 4096;
  }

  public double dutyCycleToDegrees(double dutyCyclePos) {
    return dutyCyclePos * 360;
  }

  public boolean onTargetLower(){
    return m_lowerJoint.getClosedLoopError() < ArmConstants.TOLERANCE_LOWER;
  }
  public boolean onTargetUpper(){
    return m_upperJoint.getClosedLoopError() < ArmConstants.TOLERANCE_UPPER;
  }

  public void setLowerJointFromDashboardPos(){
    m_lowerJoint.config_kP(0, SmartDashboard.getNumber("Lower P", ArmConstants.GAINS_UPPER_JOINT.kP));
    m_lowerJoint.config_kI(0, SmartDashboard.getNumber("Lower I", ArmConstants.GAINS_UPPER_JOINT.kI));
    m_lowerJoint.config_kD(0, SmartDashboard.getNumber("Lower D", ArmConstants.GAINS_UPPER_JOINT.kD));
    //m_lowerJoint.configClosedloopRamp(SmartDashboard.getNumber("Lower CLR", ArmConstants.CLOSED_LOOP_RAMP_LOWER));
    setSimplePIDLower(SmartDashboard.getNumber("Lower Setpoint", 0.0));
  }

  public void setUpperJointFromDashboardPos(){
    m_upperJoint.config_kP(0, SmartDashboard.getNumber("Upper P", ArmConstants.GAINS_LOWER_JOINT.kP));
    m_upperJoint.config_kI(0, SmartDashboard.getNumber("Upper I", ArmConstants.GAINS_LOWER_JOINT.kI));
    m_upperJoint.config_kD(0, SmartDashboard.getNumber("Upper D", ArmConstants.GAINS_LOWER_JOINT.kD));
    //m_upperJoint.configClosedloopRamp(SmartDashboard.getNumber("Upper CLR", ArmConstants.CLOSED_LOOP_RAMP_UPPER));
    setSimplePIDUpper(SmartDashboard.getNumber("Upper Setpoint", 0.0));
  }
  
  public void setLowerJointFromDashboardMotion(){
    m_lowerJoint.config_kP(0, SmartDashboard.getNumber("Lower P", ArmConstants.GAINS_LOWER_JOINT.kP));
    m_lowerJoint.config_kI(0, SmartDashboard.getNumber("Lower I", ArmConstants.GAINS_LOWER_JOINT.kI));
    m_lowerJoint.config_kD(0, SmartDashboard.getNumber("Lower D", ArmConstants.GAINS_LOWER_JOINT.kD));
    m_lowerJoint.configMotionAcceleration(SmartDashboard.getNumber("Lower Accel", ArmConstants.MOTION_ACCELERATION_LOWER));
    m_lowerJoint.configMotionCruiseVelocity(SmartDashboard.getNumber("Lower Vel", ArmConstants.MOTION_CRUISE_VELOCITY_LOWER));
    m_lowerJoint.configMotionSCurveStrength((int)SmartDashboard.getNumber("Lower Curve", ArmConstants.CURVE_SMOOTHING_LOWER));
    setMotionMagicLower(SmartDashboard.getNumber("Lower Setpoint", 0.0));
  }

  public void setUpperJointFromDashboardMotion(){
    m_upperJoint.config_kP(0, SmartDashboard.getNumber("Upper P", ArmConstants.GAINS_UPPER_JOINT.kP));
    m_upperJoint.config_kI(0, SmartDashboard.getNumber("Upper I", ArmConstants.GAINS_UPPER_JOINT.kI));
    m_upperJoint.config_kD(0, SmartDashboard.getNumber("Upper D", ArmConstants.GAINS_UPPER_JOINT.kD));
    m_upperJoint.configMotionAcceleration(SmartDashboard.getNumber("Upper Accel", ArmConstants.MOTION_ACCELERATION_UPPER));
    m_upperJoint.configMotionCruiseVelocity(SmartDashboard.getNumber("Upper Vel", ArmConstants.MOTION_CRUISE_VELOCITY_UPPER));
    m_upperJoint.configMotionSCurveStrength((int)SmartDashboard.getNumber("Upper Curve", ArmConstants.CURVE_SMOOTHING_UPPER));
    setMotionMagicUpper(SmartDashboard.getNumber("Upper Setpoint", 0.0));
  }
}
