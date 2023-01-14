// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SnapToAngle;
import frc.robot.subsystems.limelight.AlignWithConeNode;
import frc.robot.subsystems.limelight.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeOnNodeWithAutoAlign extends SequentialCommandGroup {
  /** Creates a new PlaceConeOnNode. */
  /**
   * 
   * @param arm arm subsystem
   * @param limelight limelight subsystem
   * @param drive drive subsystem
   * @param level Level of the node valid: "mid", "top"
   */
  public PlaceConeOnNodeWithAutoAlign(ArmSubsystem arm, LimelightSubsystem limelight, DriveSubsystem drive, String level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SnapToAngle(drive, 0),
      new AlignWithConeNode(limelight, drive)
      //PlaceConeOnTopNode
    );
  }
}
