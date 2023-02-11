// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.PHConstants;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private TalonFX m_lowerJoint = new WPI_TalonFX(CanConstants.LOWER_JOINT_MOTOR);
  private TalonFX m_upperJoint = new WPI_TalonFX(CanConstants.UPPER_JOINT_MOTOR);

  private DutyCycleEncoder m_upperEncoder = new DutyCycleEncoder(DIOConstants.UPPER_ENCODER_ARM);
  private DutyCycleEncoder m_lowerEncoder = new DutyCycleEncoder(DIOConstants.LOWER_ENCODER_ARM);

  private TrapezoidProfile.Constraints lowerConstraints = new TrapezoidProfile.Constraints(ArmConstants.UPPER_CRUISE,
      ArmConstants.UPPER_ACCELERATION);
  private TrapezoidProfile.Constraints upperConstraints = new TrapezoidProfile.Constraints(ArmConstants.LOWER_CRUISE,
      ArmConstants.LOWER_ACCELERATION);

  private ProfiledPIDController m_controllerLower = new ProfiledPIDController(ArmConstants.GAINS_LOWER_JOINT.kP,
      ArmConstants.GAINS_LOWER_JOINT.kI, ArmConstants.GAINS_LOWER_JOINT.kD, lowerConstraints);
  private ProfiledPIDController m_controllerUpper = new ProfiledPIDController(ArmConstants.GAINS_UPPER_JOINT.kP,
      ArmConstants.GAINS_UPPER_JOINT.kI, ArmConstants.GAINS_UPPER_JOINT.kD, upperConstraints);

  private JointConfig joint_Upper = new JointConfig(ArmConstants.UPPER_MASS, ArmConstants.UPPER_LENGTH,
      ArmConstants.UPPER_MOI, ArmConstants.UPPER_CGRADIUS, ArmConstants.UPPER_MOTOR);
  private JointConfig joint_Lower = new JointConfig(ArmConstants.LOWER_MASS, ArmConstants.LOWER_LENGTH,
      ArmConstants.LOWER_MOI, ArmConstants.LOWER_CGRADIUS, ArmConstants.LOWER_MOTOR);

  private DJArmFeedforward m_doubleJointedFeedForwards = new DJArmFeedforward(joint_Lower, joint_Upper);

  private double m_upperSetpoint;
  private double m_lowerSetpoint;
  private boolean m_writstSetpoint;
  private boolean m_upperAtSetpoint;
  private boolean m_lowerAtSetpoint;

  private Solenoid m_wrist = new Solenoid(PneumaticsModuleType.REVPH, PHConstants.WRIST_CHANNEL);
  private Solenoid m_claw = new Solenoid(PneumaticsModuleType.REVPH, PHConstants.CLAW_CHANNEL);

  public ArmSubsystem() {
    // Config Duty Cycle Range for the encoders
    m_lowerEncoder.setDutyCycleRange(ArmConstants.DUTY_CYCLE_MIN, ArmConstants.DUTY_CYCLE_MAX);
    m_upperEncoder.setDutyCycleRange(ArmConstants.DUTY_CYCLE_MIN, ArmConstants.DUTY_CYCLE_MAX);

    // Default Motors
    m_lowerJoint.configFactoryDefault(ArmConstants.TIMEOUT);
    m_upperJoint.configFactoryDefault(ArmConstants.TIMEOUT);

    // Set Neutral Mode to Brake and NeutralDeadBand to prevent need for intentional
    // stalling
    m_lowerJoint.setNeutralMode(NeutralMode.Brake);
    m_upperJoint.setNeutralMode(NeutralMode.Brake);

    m_lowerJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);
    m_upperJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);

    m_lowerJoint.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));
    m_upperJoint.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));

    m_lowerJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, ArmConstants.TIMEOUT);
    m_upperJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, ArmConstants.TIMEOUT);

    m_upperJoint.setInverted(TalonFXInvertType.CounterClockwise);
    m_lowerJoint.setInverted(TalonFXInvertType.CounterClockwise);

    m_lowerJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    m_lowerJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
    m_lowerJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    m_lowerJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

    m_upperJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    m_upperJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
    m_upperJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
    m_upperJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

    m_lowerJoint.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);
    m_upperJoint.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);

    m_lowerJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);
    m_upperJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);

    m_lowerJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);
    m_upperJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);

    m_upperJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_UPPER, ArmConstants.TIMEOUT);
    m_upperJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_UPPER, ArmConstants.TIMEOUT);
    m_lowerJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);
    m_lowerJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);

    m_controllerLower.setTolerance(ArmConstants.TOLERANCE_POS, ArmConstants.TOLERANCE_VEL);
    m_controllerUpper.setTolerance(ArmConstants.TOLERANCE_POS, ArmConstants.TOLERANCE_VEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_upperJoint.setSelectedSensorPosition(dutyCycleToCTREUnits(getUpperJointPos()), 0, ArmConstants.TIMEOUT);
    m_lowerJoint.setSelectedSensorPosition(dutyCycleToCTREUnits(getLowerJointPos()), 0, ArmConstants.TIMEOUT);

    SmartDashboard.putNumber("Upper Setpoint", m_upperSetpoint);
    SmartDashboard.putNumber("Lower Setpoint", m_lowerSetpoint);

    SmartDashboard.putBoolean("Game Peice", GamePiece.getGamePiece() == GamePieceType.Cone);
    
    if (Constants.tuningMode) {
      SmartDashboard.putBoolean("Upper at Setpoint", m_upperAtSetpoint);
      SmartDashboard.putBoolean("Lower at Setpoint", m_lowerAtSetpoint);
      SmartDashboard.putNumber("Lower Angle", getLowerJointDegrees());
      SmartDashboard.putNumber("Upper Angle", getUpperJointDegrees());
      SmartDashboard.putNumber("Lower Angle Uncorrected", dutyCycleToDegrees(getLowerJointPos()));
      SmartDashboard.putNumber("Upper Angle Uncorrected", dutyCycleToDegrees(getUpperJointPos()));
      SmartDashboard.putNumber("Lower Error", m_controllerLower.getPositionError());
      SmartDashboard.putNumber("Upper Error", m_controllerUpper.getPositionError());
      SmartDashboard.putNumber("Lower VEL Error", m_controllerLower.getVelocityError());
      SmartDashboard.putNumber("Upper VEL Error", m_controllerUpper.getVelocityError());
      SmartDashboard.putNumber("Lower Velocity Setpoint", m_controllerLower.getSetpoint().velocity);
      SmartDashboard.putNumber("Upper Velocity Setpoint", m_controllerUpper.getSetpoint().velocity);
    } else {
      SmartDashboard.clearPersistent("Lower Angle");
      SmartDashboard.clearPersistent("Upper Angle");
      SmartDashboard.clearPersistent("Upper Percent");
      SmartDashboard.clearPersistent("Lower Percent");
      SmartDashboard.clearPersistent("Upper Abs");
      SmartDashboard.clearPersistent("Lower Abs");
      SmartDashboard.clearPersistent("Upper Current");
      SmartDashboard.clearPersistent("Lower Current");
    }
  }
  public void reset() {
    m_controllerUpper.reset(getUpperJointDegrees());
    m_controllerLower.reset(getLowerJointDegrees());
    m_upperSetpoint = getUpperJointDegrees();
    m_lowerSetpoint = getLowerJointDegrees();

  }

  public void updateUpperSetpoint(double setpoint) {
    if (m_upperSetpoint != setpoint) {
      if (setpoint < 360 && setpoint > 0) {
        m_upperSetpoint = setpoint;
      }
    }
  }

  public void updateLowerSetpoint(double setpoint) {
    if (m_lowerSetpoint != setpoint) {
      if (setpoint < 360 && setpoint > 0) {
        m_lowerSetpoint = setpoint;
      }
    }
  }

  public void updateWristSetpoint(boolean setpoint){
    m_writstSetpoint = setpoint;
    m_wrist.set(m_writstSetpoint);
  }

  public void updateClawSetpoint(GamePiece.GamePieceType gamePiece){
    if(gamePiece == GamePieceType.Cone){
      m_claw.set(true);
    }

    else if(gamePiece == GamePieceType.Cube){
      m_claw.set(false);
    }
  }

  public void updateAllSetpoints(Setpoint setpoint){
    if(GamePiece.getGamePiece() == GamePieceType.Cone){
      updateUpperSetpoint(setpoint.m_upperCone);
      updateLowerSetpoint(setpoint.m_lowerCone);
      updateWristSetpoint(setpoint.wristCone);
    }
    else if (GamePiece.getGamePiece() == GamePieceType.Cube){
      updateUpperSetpoint(setpoint.m_upperCube);
      updateLowerSetpoint(setpoint.m_lowerCube);
      updateWristSetpoint(setpoint.wristCube);
    }
    updateClawSetpoint(GamePiece.getGamePiece());
  }


  public Vector<N2> calculateFeedforwards() {
                                                //Setpoint -
    Vector<N2> positionVector = VecBuilder.fill(Math.toRadians(m_lowerSetpoint + (90)), 
                                                //Setpoint + 90 - So horizantal is 0 
                                                Math.toRadians(m_upperSetpoint + (90))); 

    Vector<N2> velocityVector = VecBuilder.fill(0.0, 0.0);
    Vector<N2> accelVector = VecBuilder.fill(0.0, 0.0);
    Vector<N2> vectorFF = m_doubleJointedFeedForwards.feedforward(positionVector, velocityVector, accelVector);
    return vectorFF;
  }

  public void runUpperProfiled() {
    m_controllerUpper.setGoal(new TrapezoidProfile.State(m_upperSetpoint, 0.0));
    double pidOutput = -m_controllerUpper.calculate(getUpperJointDegrees());
    double ff = (calculateFeedforwards().get(1, 0)) / 12.0;
    System.out.println("upper ff" + (ff));
    System.out.println("Upper PID" + pidOutput);
    setPercentOutputUpper(pidOutput); // may need to negate ff voltage to get desired output
    m_upperAtSetpoint = m_controllerUpper.atGoal();
  }

  public void runLowerProfiled() {
    m_controllerLower.setGoal(new TrapezoidProfile.State(m_lowerSetpoint, 0.0));
    double pidOutput = -m_controllerLower.calculate(getLowerJointDegrees());
    double ff = -(calculateFeedforwards().get(0, 0)) / 12.0;
    System.out.println("lower ff" + (ff));
    System.out.println("Lower PID" + pidOutput);
    setPercentOutputLower(pidOutput); // may need to negate ff voltage to get desired output
    m_lowerAtSetpoint = m_controllerLower.atGoal();
  }

  public void setToCurrent() {
    m_lowerSetpoint = getLowerJointDegrees();
    m_upperSetpoint = getUpperJointDegrees();
  }

  public boolean getLowerAtSetpoint(){
    return m_lowerAtSetpoint;
  }

  public boolean getUpperAtSetpoint(){
    return m_lowerAtSetpoint;
  }

  public boolean bothJointsAtSetpoint(){
    return m_lowerAtSetpoint && m_upperAtSetpoint;
  }

  public void setPercentOutputUpper(double speed) {
    m_upperJoint.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setPercentOutputLower(double speed) {
    m_lowerJoint.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void neutralUpper() {
    m_lowerJoint.neutralOutput();
  }

  public void neutralLower() {
    m_lowerJoint.neutralOutput();
  }

  public double getLowerJointPos() {
    return m_lowerEncoder.getAbsolutePosition();
  }

  public double getUpperJointPos() {
    return m_upperEncoder.getAbsolutePosition();
  }

  public double getLowerJointDegrees() {
    return dutyCycleToDegrees(getLowerJointPos()) + ArmConstants.LOWER_ANGLE_OFFSET;
  }

  public double getUpperJointDegrees() {
    return dutyCycleToDegrees(getUpperJointPos()) + ArmConstants.UPPER_ANGLE_OFFSET;
  }

  public double dutyCycleToCTREUnits(double dutyCyclePos) {
    // 4096 units per rotation = raw sensor units for Pulse width encoder
    return dutyCyclePos * 4096;
  }

  public double dutyCycleToDegrees(double dutyCyclePos) {
    return dutyCyclePos * 360;
  }

  public void actuateWristUp(){
    m_wrist.set(true);
  }
  public void actuateWristDown(){
    m_wrist.set(false);
  }
  public void actuateClawIn(){
    m_claw.set(true);
  }
  public void actuateClawOut(){
    m_claw.set(false);
  }
}
