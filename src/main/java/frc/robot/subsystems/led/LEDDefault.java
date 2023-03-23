// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsytem;
import frc.robot.subsystems.cubeShooter.CubeShooterSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

public class LEDDefault extends CommandBase {
  /** Creates a new LEDDefault. */
  LEDSubsystem m_led;
  ClawSubsytem m_claw;
  LimelightSubsystem m_limelight;
  CubeShooterSubsystem m_shooter;
  public LEDDefault(LEDSubsystem led, ClawSubsytem claw, LimelightSubsystem limelight, CubeShooterSubsystem shooter) {
    m_led = led;
    m_claw = claw;
    m_limelight = limelight;
    m_shooter = shooter;
    addRequirements(m_led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
  }


  @Override
  public boolean runsWhenDisabled(){
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.tuningMode){
      // SmartDashboard.putBoolean("Is vision Mode", m_limelight.inVisionMode());
    }
    if(DriverStation.isDisabled()){
      m_led.setRainbow();
    }
    else if(m_claw.getClawCurrent()>=ClawConstants.CLAW_SPIKE_CURRENT || m_shooter.shooterCurrent()>1.0){
      m_led.setColor(122, 249, 240);
    }
    else if(m_limelight.inVisionMode()){
      m_led.setColor(0, 0, 0);
    }
    else if(GamePiece.getGamePiece() == GamePieceType.Cube){
      //Set color to purple
      m_led.setColor(186, 0, 255);
    }
    else if(GamePiece.getGamePiece() == GamePieceType.Cone){
      m_led.setColor(255,191,0);
    }
    else if (GamePiece.getGamePiece() == GamePieceType.None){
      m_led.setColor(255,0,0);
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
