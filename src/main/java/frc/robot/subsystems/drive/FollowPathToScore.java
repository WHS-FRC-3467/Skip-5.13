// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;

public class FollowPathToScore extends CommandBase {
  /** Creates a new FollowPathToScore. */
  DriveSubsystem m_drive;
  PathPlannerTrajectory traj;
  public FollowPathToScore(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    traj = PathPlanner.generatePath(new PathConstraints(1.0, 1.0), 
                            new PathPoint(m_drive.swerveOdometry.getPoseMeters().getTranslation(), 
                                          m_drive.swerveOdometry.getPoseMeters().getRotation(),
                                          m_drive.swerveOdometry.getPoseMeters().getRotation()),
                                          m_drive.getSelectedNode());


    PIDController thetaController = new PIDController(2.0, 0, 0);
    PIDController xController = new PIDController(1.3, 0, 0);
    PIDController yController = new PIDController(1.2, 0, 0);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //TODO: FIX this - can't have a command within a command
    new PPSwerveControllerCommand(
      traj, 
      m_drive::getPose, // Pose supplier
      SwerveConstants.SWERVE_DRIVE_KINEMATICS, // SwerveDriveKinematics
      xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      yController, // Y controller (usually the same values as X controller)
      thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      m_drive::setModuleStates,  // Module states consumer
      true, //Automatic mirroring
        m_drive // Requires this drive subsystem
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
