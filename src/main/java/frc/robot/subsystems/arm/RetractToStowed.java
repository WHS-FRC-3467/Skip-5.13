// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.Setpoint.ArmState;

public class RetractToStowed extends CommandBase {
  ArmSubsystem arm;
  boolean m_end = false;
  double count;
  boolean m_coneMidNode;
  /** Creates a new RetractToStow. */
  public RetractToStowed(ArmSubsystem arm) {
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Retracting from grid
    if (arm.getSetpoint().state.equals(ArmState.MID_NODE)){
      arm.updateAllSetpoints(ArmSetpoints.INTERMEDIATE_SETPOINT_RETRACTING_FROM_GRID);
      m_coneMidNode = true;
    } 
    else if (arm.getSetpoint().state.equals(ArmState.TOP_NODE)) {
      m_coneMidNode = false;
      arm.updateAllSetpoints(ArmSetpoints.INTERMEDIATE_SETPOINT_RETRACTING_FROM_GRID);
    }
    else if (arm.getSetpoint().state.equals(ArmState.MID_NODE_PLACED)){
      m_coneMidNode = true;
      arm.updateAllSetpoints(ArmSetpoints.INTERMEDIATE_SETPOINT_RETRACTING_FROM_GRID);

    }
    else if(arm.getSetpoint().state.equals(ArmState.TOP_NODE_PLACED)) {
      m_coneMidNode = false;
      arm.updateAllSetpoints(ArmSetpoints.INTERMEDIATE_SETPOINT_RETRACTING_FROM_GRID);
    }
    // Retracting from floor
    else if (arm.getSetpoint().state.equals(ArmState.FLOOR)) {
      m_coneMidNode = false;
      arm.updateAllSetpoints(ArmSetpoints.INTERMEDIATE_STOWED);
    }
    // No Intermediate - go directly to stow
    else {
      m_coneMidNode = false;
      arm.updateAllSetpoints(ArmSetpoints.STOWED);
      m_end = false;
    }
    count = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (arm.getSetpoint().state.equals(ArmState.INTERMEDIATE) && !arm.bothJointsAtSetpoint() && count>10) 
    { 
        //arm.updateAllSetpoints(intermediate);
        m_end = false;
    }
    else if (m_coneMidNode){
      if ((arm.bothJointsAtSetpoint() || count>200)&& !arm.getSetpoint().state.equals(ArmState.STOWED) && count>10) {
          arm.updateAllSetpoints(ArmSetpoints.STOWED_FROM_MID_CONE);
          m_end = true;
      }
      else if((arm.bothJointsAtSetpoint() && count>10) || count>200){
        arm.updateAllSetpoints(ArmSetpoints.STOWED);
        m_end = true;
      }
    }  
    else if (m_coneMidNode = false){
      if ((arm.bothJointsAtSetpoint() || count>200)&& !arm.getSetpoint().state.equals(ArmState.STOWED) && count>10) {
        arm.updateAllSetpoints(ArmSetpoints.STOWED);
        m_end = true;
      }
      else if((arm.bothJointsAtSetpoint() && count>10) || count>200){
        arm.updateAllSetpoints(ArmSetpoints.STOWED);
        m_end = true;
      }
    }   
    else{
      m_end = false;
    }
    
    System.out.println("end retract " + m_end);
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_end = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
