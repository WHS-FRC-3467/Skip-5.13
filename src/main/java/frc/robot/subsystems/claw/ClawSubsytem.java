// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class ClawSubsytem extends SubsystemBase {
  /** Creates a new ClawSubsytem. */
  private TalonSRX m_clawMotor = new TalonSRX(CanConstants.CLAW_MOTOR);
  public ClawSubsytem() {
    m_clawMotor.configFactoryDefault();
    m_clawMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    m_clawMotor.configVoltageCompSaturation(12.0);
    m_clawMotor.setNeutralMode(NeutralMode.Coast);
    m_clawMotor.configOpenloopRamp(0.0);
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Current", getClawCurrent());
    // This method will be called once per scheduler run
  }
  public void driveClaw(double speed){
    m_clawMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }
  public double getClawCurrent(){
    return m_clawMotor.getSupplyCurrent();
  }
  
}
