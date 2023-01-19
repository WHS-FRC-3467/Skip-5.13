// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArmFromDashboard extends CommandBase {
  /** Creates a new RunArmFromDashboard. */
  ArmSubsystem m_arm;
  String m_type;
  int m_typeInt;
  boolean m_end;
  /**
   *  @param type Type of position setting Valid : "LowerPID", "LowerMM", "UpperPID", "UpperMM"
   *  @param arm Armsubsystem
   */
  public RunArmFromDashboard(String type, ArmSubsystem arm) {
    m_arm = arm;
    m_type = type;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(m_type.equals("LowerPID")){
      m_typeInt =0;
    }
    if(m_type.equals("UpperPID")){
      m_typeInt =1;
    }
    if(m_type.equals("LowerMM")){
      m_typeInt =2;
    }
    if(m_type.equals("UpperMM")){
      m_typeInt =3;
    }

    m_end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_typeInt){
      case 0: 
        m_arm.setLowerJointFromDashboardPos();
        if(m_arm.onTargetLower()){
          m_typeInt = 4;
        }
        else{
          m_end = false;
        }
        break;

      case 1: 
        m_arm.setUpperJointFromDashboardPos();
        if(m_arm.onTargetUpper()){
          m_typeInt = 4;
        }
        else{
          m_end = false;
        }
        break;

      case 2:
        m_arm.setLowerJointFromDashboardMotion();
        if(m_arm.onTargetLower()){
          m_typeInt = 4;
        }
        else{
          m_end = false;
        }
        break;
      
      case 3:
        m_arm.setUpperJointFromDashboardMotion();
        if(m_arm.onTargetUpper()){
          m_typeInt = 4;
        }
        else{
          m_end = false;
        }
        break;

      case 4:
        m_end = true;
        break;

      default:
        m_end = false;
        System.out.println("Invalid Type");
        break;



    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.holdPositionLower();
    m_arm.holdPositionUpper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
