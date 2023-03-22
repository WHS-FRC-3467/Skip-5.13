// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cubeShooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */
  CubeShooterSubsystem m_shooter;
  DoubleSupplier m_speed;
  public Shoot(CubeShooterSubsystem shooter, DoubleSupplier speed) {
    m_shooter = shooter;
    m_speed = speed;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_speed.getAsDouble() > 0.2){
      m_shooter.shoot(0.25);
      m_shooter.deployShooter();
    }
    else{
      m_shooter.retractShooter();
      m_shooter.stopShooter();
    }
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
