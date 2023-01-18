// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;

public class SnapToAngle extends CommandBase {
  /** Creates a new SnapToAngle. */
  double m_angle = 0d;
  DriveSubsystem m_drive;
  private PIDController m_thetaController;

  public SnapToAngle(DriveSubsystem drive, double angle) {
    m_angle = angle;
    m_drive = drive;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_thetaController = new PIDController(SwerveConstants.GAINS_ANGLE_SNAP.kP, SwerveConstants.GAINS_ANGLE_SNAP.kI, SwerveConstants.GAINS_ANGLE_SNAP.kD);
    
    if (m_angle > 180) m_angle -= 360;
    double gyroAngle = m_drive.getYaw().getDegrees() % 360;
    gyroAngle +=  (gyroAngle > 180) ? -360 : 360;

    // Math to find shortest path
    double angleDiff = m_angle - gyroAngle;

    if (Math.abs(angleDiff) > 180) {
      if (m_angle > 0 && gyroAngle < 0) {
          m_angle -= 360;
      } 
      m_angle += 360;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroAngle = m_drive.getYaw().getDegrees() % 360;
    if (gyroAngle > 180) gyroAngle = 360 - gyroAngle;

    m_drive.drive(new Translation2d(0,0), m_thetaController.calculate(gyroAngle, -m_angle), true, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_thetaController.atSetpoint();
  }
}
