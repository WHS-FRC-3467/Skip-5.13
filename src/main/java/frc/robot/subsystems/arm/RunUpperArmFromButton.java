// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunUpperArmFromButton extends CommandBase {
  /** Creates a new RunUpperArmFromButton. */
  public double m_pos;
  public ArmSubsystem m_arm;
  public RunUpperArmFromButton(ArmSubsystem arm, double pos) {
    m_pos = pos;
    m_arm = arm;
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
    m_arm.setMotionMagicUpper(m_pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.holdPositionUpper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
