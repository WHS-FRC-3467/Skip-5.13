// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cubeShooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.PHConstants;

public class CubeShooterSubsystem extends SubsystemBase {
  /** Creates a new CubeShooterSubsystem. */
  private TalonFX m_leftShooter = new TalonFX(CanConstants.LEFT_SHOOTER);
  private TalonFX m_rightShooter = new TalonFX(CanConstants.RIGHT_SHOOTER);
  private Solenoid m_shooterPiston = new Solenoid(PneumaticsModuleType.REVPH, PHConstants.SHOOTER_CHANNEL);

  public CubeShooterSubsystem() {
    m_leftShooter.configFactoryDefault();
    m_rightShooter.configFactoryDefault();

    m_leftShooter.setNeutralMode(NeutralMode.Brake);
    m_rightShooter.setNeutralMode(NeutralMode.Brake);

    m_rightShooter.follow(m_leftShooter);
    m_rightShooter.setInverted(TalonFXInvertType.OpposeMaster);

    m_rightShooter.configVoltageCompSaturation(13.0);
    m_leftShooter.configVoltageCompSaturation(13.0);
    m_rightShooter.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.25));
    m_leftShooter.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.25));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void shoot(double speed){
    m_leftShooter.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void deployShooter(){
    m_shooterPiston.set(true);
  }

  public void retractShooter(){
    m_shooterPiston.set(false);
  }

  public void togglePiston(){
    m_shooterPiston.toggle();
  }
}
