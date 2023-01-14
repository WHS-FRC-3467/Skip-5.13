// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.util.TunableNumber;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private TalonFX m_lowerJoint = new TalonFX(CanConstants.LOWER_JOINT_MOTOR);
  private TalonFX m_upperJoint = new TalonFX(CanConstants.UPPER_JOINT_MOTOR);

  private DutyCycleEncoder m_upperEncoderAbs = new DutyCycleEncoder(DIOConstants.UPPER_ENCODER_ABS);
  private DutyCycleEncoder m_lowerEncoderAbs = new DutyCycleEncoder(DIOConstants.LOWER_ENCODER_ABS);

  private final Encoder m_upperEncoderRel = new Encoder(DIOConstants.UPPER_ENCODER_A, DIOConstants.UPPER_ENCODER_B);
  private final Encoder m_lowerEncoderRel = new Encoder(DIOConstants.LOWER_ENCODER_A, DIOConstants.LOWER_ENCODER_B);
  
  TunableNumber m_upperJointP, m_upperJointI, m_upperJointD, m_lowerJointP, m_lowerJointI, m_lowerJointD;
  TunableNumber m_upperJointSetpoint, m_lowerJointSetpoint;
  TunableNumber m_nominalFwd, m_nominalRev, m_peakFwd, m_peakRev;
  TunableNumber m_cruiseVelocityUpper, m_accelUpper, m_cruiseVelocityLower, m_accelLower;
  public ArmSubsystem() {
    m_lowerJoint.configFactoryDefault();

    m_lowerJoint.setNeutralMode(NeutralMode.Brake);
    m_lowerJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    m_lowerJoint.configNeutralDeadband(ArmConstants.kNeutralDeadband);
    m_lowerJoint.configNominalOutputForward(ArmConstants.kNominalOutputForward);
    m_lowerJoint.configNominalOutputReverse(ArmConstants.kNominalOutputReverse);
    m_lowerJoint.configPeakOutputForward(ArmConstants.kPeakOutputForward);
    m_lowerJoint.configPeakOutputReverse(ArmConstants.kPeakOutputReverse);

    m_lowerJoint.config_kP(0, ArmConstants.kGainsLowerJoint.kP);
    m_lowerJoint.config_kI(0, ArmConstants.kGainsLowerJoint.kI);
    m_lowerJoint.config_kD(0, ArmConstants.kGainsLowerJoint.kD);
    m_lowerJoint.config_kF(0, ArmConstants.kGainsLowerJoint.kF);


    m_lowerJoint.setNeutralMode(NeutralMode.Brake);
    m_lowerJoint.configMotionCruiseVelocity(ArmConstants.kMotionCruiseVelocityLower);
    m_lowerJoint.configMotionAcceleration(ArmConstants.kMotionAccelerationLower);
    m_lowerJoint.configMotionAcceleration(ArmConstants.kCurveSmoothingLower);

    m_upperJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    m_upperJoint.configNeutralDeadband(ArmConstants.kNeutralDeadband);
    m_upperJoint.configNominalOutputForward(ArmConstants.kNominalOutputForward);
    m_upperJoint.configNominalOutputReverse(ArmConstants.kNominalOutputReverse);
    m_upperJoint.configPeakOutputForward(ArmConstants.kPeakOutputForward);
    m_upperJoint.configPeakOutputReverse(ArmConstants.kPeakOutputReverse);

    m_upperJoint.config_kP(0, ArmConstants.kGainsUpperJoint.kP);
    m_upperJoint.config_kI(0, ArmConstants.kGainsUpperJoint.kP);
    m_upperJoint.config_kD(0, ArmConstants.kGainsUpperJoint.kP);
    m_upperJoint.config_kF(0, ArmConstants.kGainsUpperJoint.kP);

    m_upperJoint.configMotionCruiseVelocity(ArmConstants.kMotionCruiseVelocityUpper);
    m_upperJoint.configMotionAcceleration(ArmConstants.kMotionAccelerationUpper);
    m_upperJoint.configMotionSCurveStrength(ArmConstants.kCurveSmoothingUpper);
    m_upperJoint.setNeutralMode(NeutralMode.Brake);

    m_lowerEncoderAbs.setConnectedFrequencyThreshold(ArmConstants.kFrequency);
    m_lowerEncoderAbs.setDutyCycleRange(ArmConstants.kDutyCycleMin, ArmConstants.kDutyCycleMax);

    m_upperEncoderAbs.setConnectedFrequencyThreshold(ArmConstants.kFrequency);
    m_upperEncoderAbs.setDutyCycleRange(ArmConstants.kDutyCycleMin, ArmConstants.kDutyCycleMax);

    m_upperEncoderRel.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);
    m_lowerEncoderRel.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);

    m_upperJointP = new TunableNumber("Upper Joint P");
    m_upperJointI = new TunableNumber("Upper Joint I");
    m_upperJointD = new TunableNumber("Upper Joint D");

    m_upperJointP.setDefault(ArmConstants.kGainsUpperJoint.kP);
    m_upperJointI.setDefault(ArmConstants.kGainsUpperJoint.kI);
    m_upperJointD.setDefault(ArmConstants.kGainsUpperJoint.kD);

    m_lowerJointP = new TunableNumber("Lower Joint P");
    m_lowerJointI = new TunableNumber("Lower Joint I");
    m_lowerJointD = new TunableNumber("Lower Joint D");

    m_lowerJointP.setDefault(ArmConstants.kGainsLowerJoint.kP);
    m_lowerJointP.setDefault(ArmConstants.kGainsLowerJoint.kP);
    m_lowerJointP.setDefault(ArmConstants.kGainsLowerJoint.kP);

    m_lowerJointSetpoint = new TunableNumber("Lower Joint SetPoint");
    m_upperJointSetpoint = new TunableNumber("Upper Joint SetPoint");
    m_lowerJointSetpoint.setDefault(0);
    m_upperJointSetpoint.setDefault(0);

    m_nominalFwd = new TunableNumber("Nominal Output Forward");
    m_nominalRev = new TunableNumber("Nominal Output Reverse");

    m_nominalFwd.setDefault(ArmConstants.kNominalOutputForward);
    m_nominalRev.setDefault(ArmConstants.kNominalOutputReverse);

    m_peakFwd = new TunableNumber("Peak Output Forward");
    m_peakRev = new TunableNumber("Peak Output Reverse");

    m_peakFwd.setDefault(ArmConstants.kPeakOutputForward);
    m_peakRev.setDefault(ArmConstants.kPeakOutputReverse);


    m_cruiseVelocityLower = new TunableNumber("Motion Cruise Lower");
    m_cruiseVelocityUpper = new TunableNumber("Motion Cruise Upper");
    m_accelLower = new TunableNumber("Motion Accel Lower");
    m_accelUpper = new TunableNumber("Motion Accel Upper");

    m_cruiseVelocityLower.setDefault(ArmConstants.kMotionCruiseVelocityLower);
    m_cruiseVelocityUpper.setDefault(ArmConstants.kMotionCruiseVelocityUpper);

    m_accelLower.setDefault(ArmConstants.kMotionAcclerationLower);
    m_accelUpper.setDefault(ArmConstants.kMotionAccelerationUpper);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_upperJoint.setSelectedSensorPosition(getUpperJointPositionAbs(), 0, 10);
    m_lowerJoint.setSelectedSensorPosition(getLowerJointPositionAbs(), 0, 10);

    SmartDashboard.putNumber("Lower Joint Abs", getLowerJointPositionAbs());
    SmartDashboard.putNumber("Upper Joint Abs", getUpperJointPositionAbs());
    SmartDashboard.putNumber("Lower Joint Rel", getLowerJointPositionRel());
    SmartDashboard.putNumber("Upper Joint Rel", getUpperJointPositionRel());
    SmartDashboard.putNumber("Upper Joint Velocity", getUpperJointRPM());
    SmartDashboard.putNumber("Lower Joint Velocity", getlowerJointRPM());
  }

  public void setPercentOutputUpper(double speed){
    m_upperJoint.set(ControlMode.PercentOutput, speed);
  }

  public void setSimplePIDUpper(double position){
    m_upperJoint.set(ControlMode.Position, position);
  }

  public void setMotionMagicUpper(double position){
    m_upperJoint.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ArmConstants.kGainsUpperJoint.kF);
  }

  public void setPercentOutputLower(double speed){
    m_lowerJoint.set(ControlMode.PercentOutput, speed);
  }

  public void setSimplePIDLower(double position){
    m_lowerJoint.set(ControlMode.Position, position);
  }

  public void setMotionMagicLower(double position){
    m_lowerJoint.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ArmConstants.kGainsLowerJoint.kF);
  }

  public void holdPositionUpper(){
    m_lowerJoint.set(ControlMode.Disabled, 0);
  }
  public void holdPositionLower(){
    m_lowerJoint.set(ControlMode.Disabled, 0);
  }

  public double getLowerJointPositionAbs(){
    return m_lowerEncoderAbs.getAbsolutePosition();
  }

  public double getUpperJointPositionAbs(){
    return m_upperEncoderAbs.getAbsolutePosition();
  }

  public double getLowerJointPositionRel(){
    return m_lowerEncoderAbs.getDistance();
  }

  public double getUpperJointPositionRel(){
    return m_upperEncoderAbs.getDistance();
  }

  public double getUpperJointRPM(){
    return m_upperEncoderRel.getRate();
  }
  public double getlowerJointRPM(){
    return m_lowerEncoderRel.getRate();
  }

  public boolean onTargetLower(){
    return m_lowerJoint.getClosedLoopError() < ArmConstants.kToleranceLower;
  }
  public boolean onTargetUpper(){
    return m_lowerJoint.getClosedLoopError() < ArmConstants.kToleranceUpper;
  }


  public void setLowerJointFromDashboardPos(){
    m_lowerJoint.config_kP(0, m_lowerJointP.get());
    m_lowerJoint.config_kI(0, m_lowerJointI.get());
    m_lowerJoint.config_kD(0, m_lowerJointD.get());
    m_lowerJoint.configPeakOutputForward(m_peakFwd.get());
    m_lowerJoint.configPeakOutputReverse(m_peakRev.get());
    m_lowerJoint.configNominalOutputForward(m_nominalFwd.get());
    m_lowerJoint.configNominalOutputReverse(m_nominalRev.get());
    m_lowerJoint.configMotionAcceleration(m_accelLower.get());
    m_lowerJoint.configMotionCruiseVelocity(m_cruiseVelocityLower.get());
    setSimplePIDLower(m_lowerJointSetpoint.get());
  }

  public void setUpperJointFromDashboardPos(){
    m_upperJoint.config_kP(0, m_upperJointP.get());
    m_upperJoint.config_kI(0, m_upperJointI.get());
    m_upperJoint.config_kD(0, m_upperJointD.get());
    m_upperJoint.configPeakOutputForward(m_peakFwd.get());
    m_upperJoint.configPeakOutputReverse(m_peakRev.get());
    m_upperJoint.configNominalOutputForward(m_nominalFwd.get());
    m_upperJoint.configNominalOutputReverse(m_nominalRev.get());
    m_upperJoint.configMotionAcceleration(m_accelUpper.get());
    m_upperJoint.configMotionCruiseVelocity(m_cruiseVelocityUpper.get());
    setSimplePIDLower(m_upperJointSetpoint.get());
  }

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
