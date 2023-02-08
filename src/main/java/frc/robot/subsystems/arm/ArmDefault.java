// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmDefault extends CommandBase {
  /** Creates a new ArmDefault. */
  ArmSubsystem m_arm;
  BooleanSupplier m_joyMode;
  DoubleSupplier m_upperOutput, m_lowerOutput;
  
  public ArmDefault(ArmSubsystem arm, BooleanSupplier joyMode, DoubleSupplier upperOutput, DoubleSupplier lowerOutput) {
    m_arm = arm;
    m_upperOutput = upperOutput;
    m_lowerOutput = lowerOutput;
    m_joyMode = joyMode;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_joyMode.getAsBoolean()){
      m_arm.setToCurrent();
      m_arm.setPercentOutputUpper(m_upperOutput.getAsDouble()*0.3);
      m_arm.setPercentOutputLower(m_lowerOutput.getAsDouble()*0.3);
      m_arm.reset();
    }
    else{
      m_arm.runUpperProfiled();
      m_arm.runLowerProfiled();      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
