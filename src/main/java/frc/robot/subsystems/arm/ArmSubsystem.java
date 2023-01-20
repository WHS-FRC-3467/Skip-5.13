// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.util.TunableNumber;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private TalonFX m_lowerJoint = new TalonFX(CanConstants.LOWER_JOINT_MOTOR);
  private TalonFX m_upperJoint = new TalonFX(CanConstants.UPPER_JOINT_MOTOR);

  private DutyCycleEncoder m_upperEncoder = new DutyCycleEncoder(DIOConstants.UPPER_ENCODER_ARM);
  private DutyCycleEncoder m_lowerEncoder = new DutyCycleEncoder(DIOConstants.LOWER_ENCODER_ARM);

  
  TunableNumber m_upperJointP, m_upperJointI, m_upperJointD, m_lowerJointP, m_lowerJointI, m_lowerJointD;
  TunableNumber m_upperJointSetpoint, m_lowerJointSetpoint;
  TunableNumber m_nominalFwd, m_nominalRev, m_peakFwd, m_peakRev;
  TunableNumber m_cruiseVelocityUpper, m_accelUpper, m_cruiseVelocityLower, m_accelLower;

  public ArmSubsystem() {    
    //Config Duty Cycle Range for the encoders
    m_lowerEncoder.setDutyCycleRange(ArmConstants.DUTY_CYCLE_MIN, ArmConstants.DUTY_CYCLE_MAX);
    m_upperEncoder.setDutyCycleRange(ArmConstants.DUTY_CYCLE_MIN, ArmConstants.DUTY_CYCLE_MAX);

    //Default Motors
    m_lowerJoint.configFactoryDefault(ArmConstants.TIMEOUT);
    m_upperJoint.configFactoryDefault(ArmConstants.TIMEOUT);

    //Set Neutral Mode to Brake and NeutralDeadBand to prevent need for intentional stalling
    m_lowerJoint.setNeutralMode(NeutralMode.Brake);
    m_upperJoint.setNeutralMode(NeutralMode.Brake);

    m_lowerJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);
    m_upperJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);

    m_lowerJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, ArmConstants.TIMEOUT);
    m_upperJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, ArmConstants.TIMEOUT);

    m_upperJoint.setInverted(TalonFXInvertType.Clockwise);
    m_lowerJoint.setInverted(TalonFXInvertType.Clockwise);
    m_upperJoint.setSensorPhase(false);

    m_upperJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_UPPER);
    m_upperJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_UPPER);

    m_lowerJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_LOWER);
    m_lowerJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_LOWER);

    m_upperJoint.configForwardSoftLimitEnable(false, ArmConstants.TIMEOUT);
    m_upperJoint.configReverseSoftLimitEnable(false, ArmConstants.TIMEOUT);
    m_lowerJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);
    m_lowerJoint.configReverseSoftLimitEnable(true, ArmConstants.TIMEOUT);


    // m_lowerJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    // m_lowerJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
    m_lowerJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    m_lowerJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

    // m_upperJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    // m_upperJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
    m_upperJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    m_upperJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);


    //PID configs
    m_upperJoint.config_kP(0, ArmConstants.GAINS_UPPER_JOINT.kP, ArmConstants.TIMEOUT);
    m_upperJoint.config_kI(0, ArmConstants.GAINS_UPPER_JOINT.kP, ArmConstants.TIMEOUT);
    m_upperJoint.config_kD(0, ArmConstants.GAINS_UPPER_JOINT.kP, ArmConstants.TIMEOUT);
    m_upperJoint.config_kF(0, ArmConstants.GAINS_UPPER_JOINT.kP, ArmConstants.TIMEOUT);
    m_upperJoint.config_IntegralZone(0, ArmConstants.GAINS_UPPER_JOINT.kIzone, ArmConstants.TIMEOUT);

    m_lowerJoint.config_kP(0, ArmConstants.GAINS_LOWER_JOINT.kP, ArmConstants.TIMEOUT);
    m_lowerJoint.config_kI(0, ArmConstants.GAINS_LOWER_JOINT.kI, ArmConstants.TIMEOUT);
    m_lowerJoint.config_kD(0, ArmConstants.GAINS_LOWER_JOINT.kD, ArmConstants.TIMEOUT);
    m_lowerJoint.config_kF(0, ArmConstants.GAINS_LOWER_JOINT.kF, ArmConstants.TIMEOUT);
    m_lowerJoint.config_IntegralZone(0, ArmConstants.GAINS_LOWER_JOINT.kIzone, ArmConstants.TIMEOUT);

    //Motion Magic Configs
    m_lowerJoint.configMotionCruiseVelocity(ArmConstants.MOTION_CRUISE_VELOCITY_LOWER, ArmConstants.TIMEOUT);
    m_lowerJoint.configMotionAcceleration(ArmConstants.MOTION_ACCELERATION_LOWER, ArmConstants.TIMEOUT);
    m_lowerJoint.configMotionSCurveStrength(ArmConstants.CURVE_SMOOTHING_LOWER, ArmConstants.TIMEOUT);

    m_upperJoint.configMotionCruiseVelocity(ArmConstants.MOTION_CRUISE_VELOCITY_UPPER, ArmConstants.TIMEOUT);
    m_upperJoint.configMotionAcceleration(ArmConstants.MOTION_ACCELERATION_UPPER, ArmConstants.TIMEOUT);
    m_upperJoint.configMotionSCurveStrength(ArmConstants.CURVE_SMOOTHING_UPPER, ArmConstants.TIMEOUT);



    //Tunable Number Initializations
    // m_upperJointP = new TunableNumber("Upper Joint P");
    // m_upperJointI = new TunableNumber("Upper Joint I");
    // m_upperJointD = new TunableNumber("Upper Joint D");

    // m_lowerJointP = new TunableNumber("Lower Joint P");
    // m_lowerJointI = new TunableNumber("Lower Joint I");
    // m_lowerJointD = new TunableNumber("Lower Joint D");

    // m_upperJointP.setDefault(ArmConstants.GAINS_UPPER_JOINT.kP);
    // m_upperJointI.setDefault(ArmConstants.GAINS_UPPER_JOINT.kI);
    // m_upperJointD.setDefault(ArmConstants.GAINS_UPPER_JOINT.kD);

    // m_lowerJointP.setDefault(ArmConstants.GAINS_LOWER_JOINT.kP);
    // m_lowerJointP.setDefault(ArmConstants.GAINS_LOWER_JOINT.kP);
    // m_lowerJointP.setDefault(ArmConstants.GAINS_LOWER_JOINT.kP);

    SmartDashboard.putNumber("Upper P", ArmConstants.GAINS_UPPER_JOINT.kP);
    SmartDashboard.putNumber("Upper I", ArmConstants.GAINS_UPPER_JOINT.kI);
    SmartDashboard.putNumber("Upper D", ArmConstants.GAINS_UPPER_JOINT.kD);
    SmartDashboard.putNumber("Lower P", ArmConstants.GAINS_LOWER_JOINT.kP);
    SmartDashboard.putNumber("Lower I", ArmConstants.GAINS_LOWER_JOINT.kI);
    SmartDashboard.putNumber("Lower D", ArmConstants.GAINS_LOWER_JOINT.kD);
    SmartDashboard.putNumber("Upper Setpoint", 0.0);
    SmartDashboard.putNumber("Lower Setpoint", 0.0);

    // m_lowerJointSetpoint = new TunableNumber("Lower Joint SetPoint");
    // m_upperJointSetpoint = new TunableNumber("Upper Joint SetPoint");

    // m_lowerJointSetpoint.setDefault(0);
    // m_upperJointSetpoint.setDefault(0);

    // m_nominalFwd = new TunableNumber("Nominal Output Forward");
    // m_nominalRev = new TunableNumber("Nominal Output Reverse");

    // m_nominalFwd.setDefault(ArmConstants.NOMINAL_OUTPUT_FORWARD);
    // m_nominalRev.setDefault(ArmConstants.NOMINAL_OUTPUT_REVERSE);

    // m_peakFwd = new TunableNumber("Peak Output Forward");
    // m_peakRev = new TunableNumber("Peak Output Reverse");

    // m_peakFwd.setDefault(ArmConstants.PEAK_OUTPUT_FORWARD);
    // m_peakRev.setDefault(ArmConstants.PEAK_OUTPUT_REVERSE);


    // m_cruiseVelocityLower = new TunableNumber("Motion Cruise Lower");
    // m_cruiseVelocityUpper = new TunableNumber("Motion Cruise Upper");
    // m_accelLower = new TunableNumber("Motion Accel Lower");
    // m_accelUpper = new TunableNumber("Motion Accel Upper");

    // m_cruiseVelocityLower.setDefault(ArmConstants.MOTION_CRUISE_VELOCITY_LOWER);
    // m_cruiseVelocityUpper.setDefault(ArmConstants.MOTION_CRUISE_VELOCITY_UPPER);

    // m_accelLower.setDefault(ArmConstants.MOTION_ACCELERATION_LOWER);
    // m_accelUpper.setDefault(ArmConstants.MOTION_ACCELERATION_UPPER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_upperJoint.setSelectedSensorPosition(dutyCycleToCTREUnits(getUpperJointPos()), 0, ArmConstants.TIMEOUT);
    m_lowerJoint.setSelectedSensorPosition(dutyCycleToCTREUnits(getLowerJointPos()), 0, ArmConstants.TIMEOUT);

    if(Constants.tuningMode){
      // SmartDashboard.putNumber("Lower Joint Abs", getLowerJointPos());
      // SmartDashboard.putNumber("Upper Joint Abs", getUpperJointPos());
      SmartDashboard.putNumber("Lower Joint CTRE Units", dutyCycleToCTREUnits(getLowerJointPos()));
      SmartDashboard.putNumber("Upper Joint CTRE Units", dutyCycleToCTREUnits(getUpperJointPos()));
      SmartDashboard.putNumber("Lower Joint Absolute Angle", dutyCycleToDegrees(getLowerJointPos()));
      SmartDashboard.putNumber("Upper Joint Absolute Angle", dutyCycleToDegrees(getUpperJointPos()));

      // SmartDashboard.putNumber("Upper Joint Talon Sensor Get", m_upperJoint.getSelectedSensorPosition());
      // SmartDashboard.putNumber("Lower Joint Talon Sensor Get", m_lowerJoint.getSelectedSensorPosition());
      SmartDashboard.putNumber("Upper Joint Talon Target", m_upperJoint.getClosedLoopTarget());
      SmartDashboard.putNumber("Lower Joint Talon Target", m_lowerJoint.getClosedLoopTarget());
      SmartDashboard.putNumber("Upper Joint Talon Error", m_upperJoint.getClosedLoopError());
      SmartDashboard.putNumber("Lower Joint Talon Error", m_lowerJoint.getClosedLoopError()); 
      // SmartDashboard.putNumber("Upper P", 0.0);
      // SmartDashboard.putNumber("Upper I", 0.0);
      // SmartDashboard.putNumber("Upper D", 0.0);
      // SmartDashboard.putNumber("Lower P", 0.0);
      // SmartDashboard.putNumber("Lower I", 0.0);
      // SmartDashboard.putNumber("Lower D", 0.0);
      // SmartDashboard.putNumber("Upper Setpoint", 0.0);
      // SmartDashboard.putNumber("Lower Setpoint", 0.0);
  
    }
    else{
      SmartDashboard.clearPersistent("Lower Joint Abs");
      SmartDashboard.clearPersistent("Upper Joint Abs");
      SmartDashboard.clearPersistent("Lower Joint CTRE Units");
      SmartDashboard.clearPersistent("Upper Joint CTRE Units");
      SmartDashboard.clearPersistent("Lower Joint Absolute Angle");
      SmartDashboard.clearPersistent("Upper Joint Absolute Angle");

      SmartDashboard.clearPersistent("Upper Joint Talon Sensor Get");
      SmartDashboard.clearPersistent("Lower Joint Talon Sensor Get");
      SmartDashboard.clearPersistent("Upper Joint Talon Target");
      SmartDashboard.clearPersistent("Lower Joint Talon Target");
      SmartDashboard.clearPersistent("Upper Joint Talon Error");
      SmartDashboard.clearPersistent("Lower Joint Talon Error");
    }

  }

  public void setPercentOutputUpper(double speed){
    m_upperJoint.set(ControlMode.PercentOutput, speed);
  }

  public void setSimplePIDUpper(double position){
    System.out.println("running SimplePIDUpper");
    m_upperJoint.set(ControlMode.Position, position);
  }

  public void setMotionMagicUpper(double position){
    m_upperJoint.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ArmConstants.GAINS_UPPER_JOINT.kF);
  }

  public void setPercentOutputLower(double speed){
    m_lowerJoint.set(ControlMode.PercentOutput, speed);
  }

  public void setSimplePIDLower(double position){
    m_lowerJoint.set(ControlMode.Position, position);
  }

  public void setMotionMagicLower(double position){
    m_lowerJoint.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ArmConstants.GAINS_LOWER_JOINT.kF);
  }

  public void holdPositionUpper(){
    m_lowerJoint.set(ControlMode.Disabled, 0);
  }
  public void holdPositionLower(){
    m_lowerJoint.set(ControlMode.Disabled, 0);
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
    m_lowerJoint.config_kP(0, SmartDashboard.getNumber("Lower P", 0.0));
    m_lowerJoint.config_kI(0, SmartDashboard.getNumber("Lower I", 0.0));
    m_lowerJoint.config_kD(0, SmartDashboard.getNumber("Lower D", 0.0));
    setSimplePIDLower(SmartDashboard.getNumber("Lower Setpoint", 0.0));
  }

  public void setUpperJointFromDashboardPos(){
    m_upperJoint.config_kP(0, SmartDashboard.getNumber("Upper P", 0.0));
    m_upperJoint.config_kI(0, SmartDashboard.getNumber("Upper I", 0.0));
    m_upperJoint.config_kD(0, SmartDashboard.getNumber("Upper D", 0.0));
    setSimplePIDUpper(SmartDashboard.getNumber("Upper Setpoint", 0.0));
  }


  // public void setLowerJointFromDashboardPos(){
  //   m_lowerJoint.config_kP(0, m_lowerJointP.get());
  //   m_lowerJoint.config_kI(0, m_lowerJointI.get());
  //   m_lowerJoint.config_kD(0, m_lowerJointD.get());
  //   m_lowerJoint.configPeakOutputForward(m_peakFwd.get());
  //   m_lowerJoint.configPeakOutputReverse(m_peakRev.get());
  //   m_lowerJoint.configNominalOutputForward(m_nominalFwd.get());
  //   m_lowerJoint.configNominalOutputReverse(m_nominalRev.get());
  //   m_lowerJoint.configMotionAcceleration(m_accelLower.get());
  //   m_lowerJoint.configMotionCruiseVelocity(m_cruiseVelocityLower.get());
  //   setSimplePIDLower(m_lowerJointSetpoint.get());
  // }

  // public void setUpperJointFromDashboardPos(){
  //   m_upperJoint.config_kP(0, m_upperJointP.get());
  //   m_upperJoint.config_kI(0, m_upperJointI.get());
  //   m_upperJoint.config_kD(0, m_upperJointD.get());
  //   m_upperJoint.configPeakOutputForward(m_peakFwd.get());
  //   m_upperJoint.configPeakOutputReverse(m_peakRev.get());
  //   m_upperJoint.configNominalOutputForward(m_nominalFwd.get());
  //   m_upperJoint.configNominalOutputReverse(m_nominalRev.get());
  //   m_upperJoint.configMotionAcceleration(m_accelUpper.get());
  //   m_upperJoint.configMotionCruiseVelocity(m_cruiseVelocityUpper.get());
  //   setSimplePIDLower(m_upperJointSetpoint.get());
  // }

  public void setLowerJointFromDashboardMotion(){
    m_lowerJoint.config_kP(0, m_lowerJointP.get());
    m_lowerJoint.config_kI(0, m_lowerJointI.get());
    m_lowerJoint.config_kD(0, m_lowerJointD.get());
    m_lowerJoint.configPeakOutputForward(m_peakFwd.get());
    m_lowerJoint.configPeakOutputReverse(m_peakRev.get());
    m_lowerJoint.configNominalOutputForward(m_nominalFwd.get());
    m_lowerJoint.configNominalOutputReverse(m_nominalRev.get());
    m_lowerJoint.configMotionAcceleration(m_accelLower.get());
    m_lowerJoint.configMotionCruiseVelocity(m_cruiseVelocityLower.get());
    setMotionMagicLower(m_lowerJointSetpoint.get());
  }

  public void setUpperJointFromDashboardMotion(){
    m_upperJoint.config_kP(0, m_upperJointP.get());
    m_upperJoint.config_kI(0, m_upperJointI.get());
    m_upperJoint.config_kD(0, m_upperJointD.get());
    m_upperJoint.configPeakOutputForward(m_peakFwd.get());
    m_upperJoint.configPeakOutputReverse(m_peakRev.get());
    m_upperJoint.configNominalOutputForward(m_nominalFwd.get());
    m_upperJoint.configNominalOutputReverse(m_nominalRev.get());
    m_upperJoint.configMotionAcceleration(m_accelUpper.get());
    m_upperJoint.configMotionCruiseVelocity(m_cruiseVelocityUpper.get());
    setMotionMagicUpper(m_upperJointSetpoint.get());
  }
}
