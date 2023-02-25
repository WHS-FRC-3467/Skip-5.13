// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.ArmDefault;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.GoToPositionWithIntermediate;
import frc.robot.subsystems.arm.ScoreAndRetract;
import frc.robot.subsystems.claw.ClawSubsytem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneConeChargeWithMobility extends SequentialCommandGroup {
  /** Creates a new OneConeChargeWithMobility. */
  public OneConeChargeWithMobility(DriveSubsystem drive, ArmSubsystem arm, ClawSubsytem claw) {
        // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory path1 = PathPlanner.loadPath("ConeChargeMobility", 2.5, 1.5);
    addRequirements(arm, drive,claw);
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cone)),
          new GoToPositionWithIntermediate(arm, ArmSetpoints.TOP_NODE),
          new WaitCommand(0.1),
          new ScoreAndRetract(arm),
          drive.followTrajectoryCommand(path1, true),
          new RunCommand(drive::AutoBalance, drive).withTimeout(6)
        ),
        new ArmDefault(arm, () -> false, ()-> 0.0, ()->0.0)
      )
    );      

  }
}
