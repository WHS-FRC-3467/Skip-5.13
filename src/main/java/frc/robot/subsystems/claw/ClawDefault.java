// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawDefault extends CommandBase {
  /** Creates a new ClawDefault. */
  DoubleSupplier m_fwd, m_rev;
  ClawSubsytem m_claw;
  public ClawDefault(ClawSubsytem claw, DoubleSupplier fwd, DoubleSupplier rev){
    m_claw = claw;
    m_fwd = fwd;
    m_rev = rev;
    addRequirements(m_claw);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_fwd.getAsDouble() > 0.2){
      m_claw.driveClaw(1.0);
    }
    else if(m_rev.getAsDouble() > 0.2){
      m_claw.driveClaw(-1.0);
    }
    else if(m_fwd.getAsDouble() < 0.2 && m_rev.getAsDouble() < 0.2){
      m_claw.driveClaw(0.25);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.driveClaw(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
