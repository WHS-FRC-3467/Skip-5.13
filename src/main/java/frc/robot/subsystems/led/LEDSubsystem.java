// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private CANdle m_candle = new CANdle(CanConstants.LED_CANDLE);
  public LEDSubsystem() {
    m_candle.configBrightnessScalar(0.75);
    m_candle.configLEDType(LEDStripType.RGB);

  }

  @Override
  public void periodic() {    
    // This method will be called once per scheduler run
  }
  public void setColor(int r, int g, int b){
    m_candle.setLEDs(r, g, b);
  }
}
