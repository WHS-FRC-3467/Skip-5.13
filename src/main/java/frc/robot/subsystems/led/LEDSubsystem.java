// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private CANdle m_candle = new CANdle(CanConstants.LED_CANDLE);
  public LEDSubsystem() {
    m_candle.configBrightnessScalar(0.75);
    m_candle.configLEDType(LEDStripType.GRB);
    m_candle.configV5Enabled(true);
    // m_candle.configLOSBehavior(true);
  }

  @Override
  public void periodic() {    
    // This method will be called once per scheduler run
  }
  public void setColor(int r, int g, int b){
    m_candle.clearAnimation(0);
    m_candle.setLEDs(r, g, b, 255, 0, 20);
  }
  public void setRainbow(){
    m_candle.animate(new RainbowAnimation(0.5,0.1,20));
  }
  //Texas mode inspired by 6328
  public void setTexasMode(){
    m_candle.clearAnimation(0);
    //yellow
    m_candle.setLEDs(230, 230, 0, 0, 0, 1);
    //white
    m_candle.setLEDs(255, 255, 255, 255, 1, 1);
    //purple
    m_candle.setLEDs(138, 0, 230, 0, 2, 1);
    //black
    m_candle.setLEDs(0, 0, 0, 0, 3, 2);
    //Purple
    m_candle.setLEDs(138, 0, 239, 0, 5, 1);
    //white 
    m_candle.setLEDs(255, 255, 255, 255, 6, 1);
    //yellow
    m_candle.setLEDs(230, 230, 0, 0, 7, 1);
    //Blue
    m_candle.setLEDs(0, 50, 255, 0, 8, 2);
    //pink
    m_candle.setLEDs(255, 51, 204, 0, 10, 2);
    //white
    m_candle.setLEDs(255, 255, 255, 255, 12, 2);
    //pink
    m_candle.setLEDs(255, 51, 204, 0, 14, 1);
    //blue
    m_candle.setLEDs(0, 50, 255, 0, 15, 1);
  }
}
