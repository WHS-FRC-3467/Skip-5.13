// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.Setpoint.ArmState;
import frc.robot.subsystems.arm.Setpoint.ClawState;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

public class ScoreOnGrid extends CommandBase {
  /** Creates a new ScoreOnGrid. */
  ArmSubsystem m_arm;
  boolean m_end;
  double count;
  boolean upper;
  public ScoreOnGrid(ArmSubsystem arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(GamePiece.getGamePiece() == GamePieceType.Cube){
      m_end = true;
    }
    else if (m_arm.getSetpoint().state.equals(ArmState.TOP_NODE)){
      m_arm.updateAllSetpoints(ArmSetpoints.TOP_NODE_PLACED);
      m_end = false;
    }
    else if(m_arm.getSetpoint().state.equals(ArmState.MID_NODE)){
      m_arm.updateAllSetpoints(ArmSetpoints.MID_NODE_PLACED);
      m_end = false;
    }
    else{
      m_end = true;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("both at setpoint" + m_arm.bothJointsAtSetpoint());
    System.out.println("end" + m_end);
    if((m_arm.bothJointsAtSetpoint() && count > 10) || count>200){
      if(m_arm.getSetpoint().state.equals(ArmState.TOP_NODE_PLACED)){
        m_arm.updateAllSetpoints(ArmSetpoints.TOP_NODE_PLACED_AND_OPEN);
      }
      else if (m_arm.getSetpoint().state.equals(ArmState.MID_NODE_PLACED)){
        m_arm.updateAllSetpoints(ArmSetpoints.MID_NODE_PLACED_AND_OPEN);
      }
      m_end = true;
    }
    else{
      m_end = false;
    }
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.updateClawSetpoint(ClawState.OUT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
