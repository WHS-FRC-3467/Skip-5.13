// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;

public class SnapToAngle extends CommandBase {
  /** Creates a new SnapToAngle. */
  DriveSubsystem m_drive;
  double m_angle;
  PIDController m_thetaController;
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
        
    m_thetaController.enableContinuousInput(-180, 180);
    m_thetaController.setSetpoint(m_angle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {      
    double rotationVal = m_thetaController.calculate(-(MathUtil.inputModulus(m_drive.getYaw().getDegrees(), -180, 180)), m_thetaController.getSetpoint());
    m_drive.drive(
      new Translation2d(0.0, 0.0), 
      rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY, 
      true,
      false,
      true);

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
