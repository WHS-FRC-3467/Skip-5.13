// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.Setpoint.ArmState;
import frc.robot.subsystems.arm.Setpoint.ClawState;

public class RetractFromFloor extends CommandBase {
  ArmSubsystem arm;
  Setpoint intermediate = new Setpoint(
    ArmSetpoints.STOWED.lowerCone,
    ArmSetpoints.STOWED.upperCone,
    true,
    ClawState.IN,
    ArmSetpoints.STOWED.lowerCube,
    ArmSetpoints.STOWED.upperCube,
    true,
    ClawState.OUT,
    ArmState.INTERMEDIATE);
  boolean m_end = false;
  double count;
  /** Creates a new RetractToStow. */
  public RetractFromFloor(ArmSubsystem arm) {
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // Retracting from floor

      intermediate = new Setpoint(
          ArmSetpoints.STOWED.lowerCone,
          ArmSetpoints.STOWED.upperCone,
          true,
          ClawState.IN,
          ArmSetpoints.STOWED.lowerCube,
          ArmSetpoints.STOWED.upperCube,
          true,
          ClawState.OUT,
          ArmState.INTERMEDIATE);

      arm.updateAllSetpoints(intermediate);
    
        count = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (arm.getSetpoint().state.equals(ArmState.INTERMEDIATE) && !arm.bothJointsAtSetpoint() && count>10) 
    { 
        arm.updateAllSetpoints(intermediate);
        m_end = false;
    }     
    else if ((arm.bothJointsAtSetpoint() || count>200)&& !arm.getSetpoint().state.equals(ArmState.STOWED) && count>10) {
        arm.updateAllSetpoints(ArmSetpoints.STOWED);
        m_end = true;
    }
    else if((arm.bothJointsAtSetpoint() && count>10) || count>200){
      arm.updateAllSetpoints(ArmSetpoints.STOWED);
      m_end = true;
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
