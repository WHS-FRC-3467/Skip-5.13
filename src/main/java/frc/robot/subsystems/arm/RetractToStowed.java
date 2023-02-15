// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.Setpoint.ArmState;

public class RetractToStowed extends CommandBase {
  ArmSubsystem arm;
  Setpoint intermediate;
  boolean m_end = false;
  double count;
  /** Creates a new RetractToStow. */
  public RetractToStowed(ArmSubsystem arm) {
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Retracting from grid
    if (arm.getSetpoint().state.equals(ArmState.MID_NODE) || 
                arm.getSetpoint().state.equals(ArmState.MID_NODE_PLACED) || 
                arm.getSetpoint().state.equals(ArmState.TOP_NODE) || 
                arm.getSetpoint().state.equals(ArmState.TOP_NODE_PLACED)) {

      intermediate = new Setpoint(
          ArmSetpoints.INTERMEDIATE_LOWER_POSITION,
          arm.getSetpoint().upperCone * 0.5,
          arm.getSetpoint().wristCone,
          ArmSetpoints.INTERMEDIATE_LOWER_POSITION,
          arm.getSetpoint().lowerCube * 0.5,
          arm.getSetpoint().wristCube,
          ArmState.INTERMEDIATE);

      arm.updateAllSetpoints(intermediate);
    }
    // Retracting from floor
    else if (arm.getSetpoint().state.equals(ArmState.FLOOR)) {

      intermediate = new Setpoint(
          ArmSetpoints.STOWED.lowerCone,
          ArmSetpoints.STOWED.upperCone + 20,
          true,
          ArmSetpoints.STOWED.lowerCube,
          ArmSetpoints.STOWED.upperCube + 20,
          true,
          ArmState.INTERMEDIATE);

      arm.updateAllSetpoints(intermediate);
    }
    // No Intermediate - go directly to stow
    else {
      arm.updateAllSetpoints(ArmSetpoints.STOWED);
      m_end = true;
    }
    count = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (!arm.getSetpoint().state.equals(ArmState.STOWED) && !arm.getSetpoint().state.equals(ArmState.INTERMEDIATE)) 
    { 
          // if joymode is entered or another setpoint is set, stop
          arm.updateAllSetpoints(intermediate);
         m_end = false;
    }     
    else if (arm.bothJointsAtSetpoint() && !arm.getSetpoint().state.equals(ArmState.STOWED) && count>50) {
      arm.updateAllSetpoints(ArmSetpoints.STOWED);
      m_end = true;
    }  
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
