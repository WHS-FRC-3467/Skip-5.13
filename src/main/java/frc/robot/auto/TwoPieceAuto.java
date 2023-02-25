// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.GoToPositionWithIntermediate;
import frc.robot.subsystems.arm.RetractToStowed;
import frc.robot.subsystems.arm.ScoreAndRetract;
import frc.robot.subsystems.claw.ClawSubsytem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.limelight.AlignWithGridAprilAuto;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAuto extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
  public TwoPieceAuto(DriveSubsystem drive, ArmSubsystem arm, ClawSubsytem claw, LimelightSubsystem limelight) {
    PathPlannerTrajectory path1 = PathPlanner.loadPath("TwoPiecePart1", new PathConstraints(5.0, 7.0));
    PathPlannerTrajectory path2 = PathPlanner.loadPath("TwoPiecePart2", new PathConstraints(2.0, 2.0));
    PathPlannerTrajectory path3 = PathPlanner.loadPath("TwoPiecePart3", new PathConstraints(5.0, 7.0));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cone)),
        new WaitCommand(0.1),
        new GoToPositionWithIntermediate(arm, ArmSetpoints.TOP_NODE),
        new ScoreAndRetract(arm),
        Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cube)),
        drive.followTrajectoryCommand(path1, true),
        Commands.runOnce(() -> arm.updateAllSetpoints(ArmSetpoints.FLOOR)),
        drive.followTrajectoryCommand(path2, false).raceWith(Commands.run(()-> claw.driveClaw(0.8), claw)),
        new WaitCommand(0.05),
        drive.followTrajectoryCommand(path3, false).raceWith(new RetractToStowed(arm)),
        new AlignWithGridAprilAuto(limelight, drive).withTimeout(3.0),
        new GoToPositionWithIntermediate(arm, ArmSetpoints.TOP_NODE),
        new RetractToStowed(arm).raceWith(Commands.run(()-> claw.driveClaw(-0.5)))
    );
  }
}
