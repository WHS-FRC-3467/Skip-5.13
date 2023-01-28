// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;

public class HoldPosition extends CommandBase {

  /** Creates a new HoldPosition. */
  ArmSubsystem m_arm;
  double m_lowerStart, m_upperStart;

  PIDController m_controllerLower = new PIDController(ArmConstants.GAINS_LOWER_JOINT.kP, ArmConstants.GAINS_LOWER_JOINT.kI, ArmConstants.GAINS_LOWER_JOINT.kD);
  PIDController m_controllerUpper = new PIDController(ArmConstants.GAINS_UPPER_JOINT.kP, ArmConstants.GAINS_UPPER_JOINT.kI, ArmConstants.GAINS_UPPER_JOINT.kD);

  public HoldPosition(ArmSubsystem arm) {
    m_arm = arm;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lowerStart = m_arm.getLowerJointDegrees();
    m_upperStart = m_arm.getUpperJointDegrees();

    m_controllerUpper.setSetpoint(m_upperStart);
    m_controllerUpper.disableContinuousInput();
    m_controllerUpper.setTolerance(0.0);

    m_controllerLower.setSetpoint(m_lowerStart);
    m_controllerLower.disableContinuousInput();
    m_controllerLower.setTolerance(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upperEncoderVal = m_arm.getUpperJointDegrees();
    double upperOutput = m_controllerUpper.calculate(upperEncoderVal, m_upperStart);
    m_arm.setPercentOutputUpper(upperOutput);

    double lowerEncoderVal = m_arm.getLowerJointDegrees();
    double lowerOutput = m_controllerLower.calculate(lowerEncoderVal, m_lowerStart);
    m_arm.setPercentOutputLower(lowerOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
