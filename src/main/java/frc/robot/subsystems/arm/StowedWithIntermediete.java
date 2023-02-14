// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowedWithIntermediete extends SequentialCommandGroup {
  /** Creates a new FloorPickUp. */
  public StowedWithIntermediete(ArmSubsystem arm, Setpoint setpoint) {
    Setpoint intermediete = new Setpoint(setpoint.lowerCone, setpoint.upperCone + 20 , true, 
                                        setpoint.lowerCube, setpoint.upperCube + 20, true);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new InstantCommand(()-> arm.updateAllSetpoints(intermediete)),
      new WaitCommand(1.0),
      new InstantCommand( ()-> arm.updateAllSetpoints(setpoint)),
      new WaitCommand(1.0)
    );
  }
}
