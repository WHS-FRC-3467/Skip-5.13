// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  DriveSubsystem m_drive;
  PIDController m_thetaController;
  PIDController m_balanceController;
  boolean m_angled, m_balanced, m_end;
  double m_angle;
  public AutoBalance(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_balanceController = new PIDController(SwerveConstants.GAINS_BALANCE.kP, SwerveConstants.GAINS_BALANCE.kI, SwerveConstants.GAINS_BALANCE.kD);
    m_balanceController.setTolerance(SwerveConstants.BALANCE_TOLLERANCE);

    m_thetaController = new PIDController(SwerveConstants.GAINS_BALANCE.kP, SwerveConstants.GAINS_BALANCE.kI, SwerveConstants.GAINS_BALANCE.kD);
    m_thetaController.setTolerance(SwerveConstants.SNAP_TOLLERANCE);

    double gyroAngle = m_drive.getYaw().getDegrees() % 360;
    gyroAngle +=  (gyroAngle > 180) ? -360 : 360;

    if(gyroAngle>=270 || gyroAngle<=90){
      m_angle = 0;
    }
    else{
      m_angle = 180;
    }

    if (m_angle > 180) m_angle -= 360;
    // Math to find shortest path
    double angleDiff = m_angle - gyroAngle;

    if (Math.abs(angleDiff) > 180) {
      if (m_angle > 0 && gyroAngle < 0) {
          m_angle -= 360;
      } 
      m_angle += 360;
    }
    m_end = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_thetaController.atSetpoint() == false){
      double gyroAngle = m_drive.getYaw().getDegrees() % 360;
      if (gyroAngle > 180) gyroAngle = 360 - gyroAngle;
  
      m_drive.drive(new Translation2d(0,0), m_thetaController.calculate(gyroAngle, -m_angle), true, true);  

      m_end = false;
    }
    else if(m_thetaController.atSetpoint() ==true && m_balanceController.atSetpoint() == false){

      m_end = false;
    }
    else if(m_thetaController.atSetpoint() ==true && m_balanceController.atSetpoint() == true){
      m_end = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
