// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class ClawSubsytem extends SubsystemBase {
  /** Creates a new ClawSubsytem. */
  private TalonSRX m_clawMotor = new TalonSRX(CanConstants.CLAW_MOTOR);
  // private Solenoid m_clawJoint = new Solenoid(PneumaticsModuleType.REVPH, PHConstants.CLAW_JOINT_CHANNEL);
  public ClawSubsytem() {
    m_clawMotor.configFactoryDefault();
    m_clawMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));
    m_clawMotor.setNeutralMode(NeutralMode.Brake);
    m_clawMotor.configOpenloopRamp(0.2);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void driveClaw(double speed){
    m_clawMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }
  // public void actuateClawUp(){
  //   m_clawJoint.set(true);
  // }
  // public void actuateClawDown(){
  //   m_clawJoint.set(false);
  // }
  // public void toggleClaw(){
  //   m_clawJoint.toggle();
  // }
}
