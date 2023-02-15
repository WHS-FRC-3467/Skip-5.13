// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private CANdle m_candle = new CANdle(CanConstants.LED_CANDLE);
  public LEDSubsystem() {
    m_candle.configBrightnessScalar(0.75);
    m_candle.configLEDType(LEDStripType.RGB);

  }

  @Override
  public void periodic() {
    if(GamePiece.getGamePiece() == GamePieceType.Cube){
      //Set color to purple
      setColor(127, 0, 255);
    }
    else if(GamePiece.getGamePiece() == GamePieceType.Cone){
      setColor(255,191,0);
    }
    else if (GamePiece.getGamePiece() == GamePieceType.None){
      setColor(255,0,0);
    }
    
    // This method will be called once per scheduler run
  }
  public void setColor(int r, int g, int b){
    m_candle.setLEDs(r, g, b);
  }
}
