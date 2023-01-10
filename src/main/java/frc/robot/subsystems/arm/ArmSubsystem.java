// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private TalonFX m_lowerJoint = new TalonFX(CanConstants.LOWER_JOINT_MOTOR);
  private TalonFX m_upperJoint = new TalonFX(CanConstants.UPPER_JOINT_MOTOR);

  public ArmSubsystem() {
    m_lowerJoint.configFactoryDefault();

    m_lowerJoint.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
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

    m_upperJoint.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentOutPutUpper(double speed){
    m_upperJoint.set(ControlMode.PercentOutput, speed);
  }

  public void setSimplePIDUpper(double position){
    m_upperJoint.set(ControlMode.Position, position);
  }

  public void setMotionMagicUpper(double position){
    m_upperJoint.set(ControlMode.MotionMagic, position);
  }

  public void setPercentOutPutLower(double speed){
    m_lowerJoint.set(ControlMode.PercentOutput, speed);
  }

  public void setSimplePIDLower(double position){
    m_lowerJoint.set(ControlMode.Position, position);
  }

  public void setMotionMagicLower(double position){
    m_lowerJoint.set(ControlMode.MotionMagic, position);
  }

  
}
