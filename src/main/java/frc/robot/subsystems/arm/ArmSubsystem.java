// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private TalonFX m_lowerJoint = new TalonFX(CanConstants.LOWER_JOINT_MOTOR);
  private TalonFX m_upperJoint = new TalonFX(CanConstants.UPPER_JOINT_MOTOR);

  private DutyCycleEncoder m_upperEncoder = new DutyCycleEncoder(DIOConstants.UPPER_ENCODER_ABS);
  private DutyCycleEncoder m_lowerEncoder = new DutyCycleEncoder(DIOConstants.LOWER_ENCODER_ABS);

  public ArmSubsystem() {
    m_lowerJoint.configFactoryDefault();

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


    m_upperJoint.configMotionCruiseVelocity(ArmConstants.kMotionCruiseVelocityLower);
    m_upperJoint.configMotionAcceleration(ArmConstants.kMotionAccelerationLower);

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

    m_lowerEncoder.setConnectedFrequencyThreshold(ArmConstants.kFrequency);
    m_lowerEncoder.setDutyCycleRange(ArmConstants.kDutyCycleMin, ArmConstants.kDutyCycleMax);

    m_upperEncoder.setConnectedFrequencyThreshold(ArmConstants.kFrequency);
    m_upperEncoder.setDutyCycleRange(ArmConstants.kDutyCycleMin, ArmConstants.kDutyCycleMax);

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

  }

  public void setPercentOutputUpper(double speed){
    m_upperJoint.set(ControlMode.PercentOutput, speed);
  }

  public void setSimplePIDUpper(double position){
    m_upperJoint.set(ControlMode.Position, position);
  }

  public void setMotionMagicUpper(double position){
    m_upperJoint.set(ControlMode.MotionMagic, position);
  }

  public void setPercentOutputLower(double speed){
    m_lowerJoint.set(ControlMode.PercentOutput, speed);
  }

  public void setSimplePIDLower(double position){
    m_lowerJoint.set(ControlMode.Position, position);
  }

  public void setMotionMagicLower(double position){
    m_lowerJoint.set(ControlMode.MotionMagic, position);
  }

  public double getLowerJointPositionAbs(){
    return m_lowerEncoder.getAbsolutePosition();
  }

  public double getUpperJointPositionAbs(){
    return m_upperEncoder.getAbsolutePosition();
  }

  public double getLowerJointPositionRel(){
    return m_lowerEncoder.getDistance();
  }

  public double getUpperJointPositionRel(){
    return m_upperEncoder.getDistance();
  }
}
