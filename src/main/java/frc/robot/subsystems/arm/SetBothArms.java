// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;

public class SetBothArms extends CommandBase {
  /** Creates a new SetBothArms. */
  ArmSubsystem m_arm;
  double m_lowerSepoint, m_upperSetpoint;
  boolean m_end;

  PIDController m_controllerLower = new PIDController(ArmConstants.GAINS_LOWER_JOINT.kP, ArmConstants.GAINS_LOWER_JOINT.kI, ArmConstants.GAINS_LOWER_JOINT.kD);
  PIDController m_controllerUpper = new PIDController(ArmConstants.GAINS_UPPER_JOINT.kP, ArmConstants.GAINS_UPPER_JOINT.kI, ArmConstants.GAINS_UPPER_JOINT.kD);

  public SetBothArms(ArmSubsystem arm, double lowerSetpoint, double upperSetpoint) {
    m_lowerSepoint = lowerSetpoint;
    m_upperSetpoint = upperSetpoint;
    m_arm = arm;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controllerUpper.setSetpoint(m_upperSetpoint);
    m_controllerUpper.disableContinuousInput();
    m_controllerUpper.setTolerance(ArmConstants.TOLERANCE_UPPER);

    m_controllerLower.setSetpoint(m_lowerSepoint);
    m_controllerLower.disableContinuousInput();
    m_controllerLower.setTolerance(ArmConstants.TOLERANCE_LOWER);

    m_end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upperEncoderVal = m_arm.getUpperJointDegrees();
    double upperOutput = m_controllerUpper.calculate(upperEncoderVal, m_upperSetpoint);
    m_arm.setPercentOutputUpper(upperOutput);

    double lowerEncoderVal = m_arm.getLowerJointDegrees();
    double lowerOutput = m_controllerUpper.calculate(lowerEncoderVal, m_lowerSepoint);
    m_arm.setPercentOutputLower(lowerOutput);

    if(m_controllerUpper.atSetpoint() && m_controllerLower.atSetpoint()){
      m_end = true;
    }
    else{
      m_end = false;
    }

    System.out.println("Upper Error " + m_controllerUpper.getPositionError());
    System.out.println("Lower Error " + m_controllerLower.getPositionError());
    System.out.println("Upper at setpoint" + m_controllerUpper.atSetpoint());
    System.out.println("Lower at setpoint" + m_controllerLower.atSetpoint());
    System.out.println("m_end " + m_end);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.holdPositionLower();
    m_arm.holdPositionUpper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
