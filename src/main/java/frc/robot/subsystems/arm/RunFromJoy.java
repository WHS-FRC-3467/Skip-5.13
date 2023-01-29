// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunFromJoy extends CommandBase {
  /** Creates a new RunFromJoy. */
  DoubleSupplier m_lowerVal, m_upperVal;
  ArmSubsystem m_arm;
  public RunFromJoy(ArmSubsystem arm, DoubleSupplier lowerVal, DoubleSupplier upperVal) {
    m_arm = arm;
    m_lowerVal = lowerVal;
    m_upperVal = upperVal;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_arm.setRunFromSticks(true);
      m_arm.setPercentOutputUpper(m_upperVal.getAsDouble() * 0.5);    
      m_arm.setPercentOutputLower(-m_lowerVal.getAsDouble() * 0.5);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.updateUpperSetpoint(m_arm.getUpperJointDegrees());
    m_arm.updateLowerSetpoint(m_arm.getLowerJointDegrees());
    m_arm.setRunFromSticks(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
