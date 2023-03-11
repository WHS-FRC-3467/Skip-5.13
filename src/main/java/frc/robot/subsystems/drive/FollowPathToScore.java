// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FollowPathToScore extends SequentialCommandGroup {
  /** Creates a new FollowPathToScore. */
  public FollowPathToScore(DriveSubsystem drive) {
    
    addRequirements(drive);
    addCommands(
      drive.followTrajectoryCommand(drive.pathToScore(), false)
    );
    // Use addRequirements() here to declare subsystem dependencies.
  }

  
}
