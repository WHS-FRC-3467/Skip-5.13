// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;

public class ArmRioPID extends CommandBase {
  /** Creates a new ArmRioPID. */
  ArmSubsystem m_arm;
  double m_position;

  boolean m_end;
  TrapezoidProfile.Constraints m_trapProfile = new TrapezoidProfile.Constraints(ArmConstants.MOTION_CRUISE_VELOCITY_UPPER, ArmConstants.MOTION_ACCELERATION_UPPER);

  private ProfiledPIDController m_controller = new ProfiledPIDController(ArmConstants.UPPER_P, ArmConstants.UPPER_I, ArmConstants.UPPER_D, m_trapProfile);
  
  public ArmRioPID(ArmSubsystem arm, double position) {
    m_arm = arm;
    m_position = position;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setGoal(m_position);
    m_controller.disableContinuousInput();
    m_controller.setTolerance(ArmConstants.TOLERANCE_UPPER);
    //m_controller.reset(m_arm.dutyCycleToDegrees(m_arm.getLowerJointPos()));
    m_end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderVal = m_arm.dutyCycleToDegrees(m_arm.getUpperJointPos());
    double outPut = m_controller.calculate(encoderVal);
    double error = m_controller.getPositionError();
    m_arm.setPercentOutputUpper(m_controller.calculate(m_arm.dutyCycleToDegrees(m_arm.getUpperJointPos())));
    
    System.out.println("Encoder Value Degrees " + m_arm.dutyCycleToDegrees(m_arm.getUpperJointPos()));
    System.out.println("PID Output " + outPut);
    System.out.println("Arm error " + error);
    m_end = m_controller.atGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.holdPositionUpper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
