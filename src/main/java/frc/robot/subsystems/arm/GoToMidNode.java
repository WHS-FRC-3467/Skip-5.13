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

public class GoToMidNode extends CommandBase {
  /** Creates a new GoToGridPosition. */
  ArmSubsystem m_arm;
  Setpoint m_setpoint;
  Boolean m_end;
  Boolean useIntermediete;
  double count;
  Setpoint intermediateSetpoint;
  Boolean intermediete;
  public GoToMidNode(ArmSubsystem arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(GamePiece.getGamePiece() == GamePieceType.Cone){
      useIntermediete = true;
    }
    else if (GamePiece.getGamePiece() == GamePieceType.Cube){
      useIntermediete = false;
    }
    
    intermediateSetpoint = new Setpoint(ArmSetpoints.INTERMEDIATE_LOWER_POSITION_SCORING, ArmSetpoints.MID_NODE.upperCone * 0.55, ArmSetpoints.MID_NODE.wristCone, ClawState.IN,
                                        ArmSetpoints.INTERMEDIATE_LOWER_POSITION_SCORING, (ArmSetpoints.MID_NODE.upperCube) * 0.55, ArmSetpoints.MID_NODE.wristCube, ClawState.OUT,
                                        ArmState.INTERMEDIATE);
    if(useIntermediete){
      m_arm.updateAllSetpoints(intermediateSetpoint);
      intermediete = true;
    }
    else{
      m_arm.updateAllSetpoints(ArmSetpoints.MID_NODE);
      intermediete = false;
    }
    count = 1;
    m_end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intermediateSetpoint = new Setpoint(ArmSetpoints.INTERMEDIATE_LOWER_POSITION_SCORING, ArmSetpoints.MID_NODE.upperCone * 0.55, ArmSetpoints.MID_NODE.wristCone, ClawState.IN,
                                        ArmSetpoints.INTERMEDIATE_LOWER_POSITION_SCORING, (ArmSetpoints.MID_NODE.upperCube) * 0.55, ArmSetpoints.MID_NODE.wristCube, ClawState.OUT,
                                        ArmState.INTERMEDIATE);

    if(useIntermediete && count>10){
      if(intermediete  && count>10){
        if(m_arm.bothJointsAtSetpoint()  && count>10){
          intermediete = false;
          m_arm.updateAllSetpoints(ArmSetpoints.MID_NODE);
          m_end = false;
          System.out.println("Case A");
        }
        else{
          intermediete = true;
          m_arm.updateAllSetpoints(intermediateSetpoint);
          m_end = false;
          System.out.println("Case B");
        }
      }
      else{
        if(m_arm.bothJointsAtSetpoint() && count>10){
          m_end = true;
          m_arm.updateAllSetpoints(ArmSetpoints.MID_NODE);
          System.out.println("Case C");
        }
        else{
          System.out.println("Case D");
          m_end = false;
          m_arm.updateAllSetpoints(ArmSetpoints.MID_NODE);
        }
      }
    }
    else{
      if(m_arm.bothJointsAtSetpoint() && count>10){
        m_end = true;
        m_arm.updateAllSetpoints(ArmSetpoints.MID_NODE);
        System.out.println("Case E");
      }
      else{
        m_end = false;
        m_arm.updateAllSetpoints(ArmSetpoints.MID_NODE);
        System.out.println("Case F");
      }
    }
    System.out.println("MID END" + m_end);
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
