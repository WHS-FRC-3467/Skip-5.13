package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 m_gyro;

    public Field2d m_field;
    private PIDController m_balancePID = new PIDController(SwerveConstants.GAINS_BALANCE.kP, SwerveConstants.GAINS_BALANCE.kI, SwerveConstants.GAINS_BALANCE.kD);

    public SwerveAutoBuilder m_autoBuilder;
    public DriveSubsystem() {
        m_gyro = new Pigeon2(CanConstants.PIGEON2, "drive");
        m_gyro.configFactoryDefault();
        zeroGyro();
        m_field = new Field2d();
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.constants),
            new SwerveModule(1, SwerveConstants.Mod1.constants),
            new SwerveModule(2, SwerveConstants.Mod2.constants),
            new SwerveModule(3, SwerveConstants.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_DRIVE_KINEMATICS, getYaw(), getModulePositions());
    }

    @Override
    public void periodic(){
        m_balancePID.setTolerance(SwerveConstants.BALANCE_TOLLERANCE);
        double pidOutput = m_balancePID.calculate(getRoll(), 0);
        SmartDashboard.putNumber("Balance PID", pidOutput);
        SmartDashboard.putNumber("Robot Pitch", getPitch());
        SmartDashboard.putNumber("Robot Roll", getRoll());

        SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().name());

        swerveOdometry.update(getYaw(), getModulePositions()); 

        m_field.setRobotPose(swerveOdometry.getPoseMeters());

        SmartDashboard.putData("Feild", m_field);
        if(Constants.tuningMode){
            for(SwerveModule mod : mSwerveMods){
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            } 
        }
        else{
            for(SwerveModule mod : mSwerveMods){
                SmartDashboard.clearPersistent("Mod " + mod.moduleNumber + " Cancoder");
                SmartDashboard.clearPersistent("Mod " + mod.moduleNumber + " Integrated");
                SmartDashboard.clearPersistent("Mod " + mod.moduleNumber + " Velocity");    
            }   
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean isTeleop) {
        SwerveModuleState[] swerveModuleStates =
        SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop, isTeleop);
        }

        SmartDashboard.putNumber("X Translation", translation.getX());
        SmartDashboard.putNumber("Y Translation", translation.getY());
        SmartDashboard.putNumber("Rotation Value", rotation);
    }    

    public void stopDrive(){
        drive(new Translation2d(0, 0), 0, false, true, true);
    }


    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false, false);
        }
    }    

    public Pose2d getPose() {
        
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        m_gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw()) : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public double getPitch(){
        return m_gyro.getPitch();
    }

    public double getRoll(){
        return m_gyro.getRoll();
    }

   
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void AutoBalance(){
        m_balancePID.setTolerance(SwerveConstants.BALANCE_TOLLERANCE);
        double pidOutput;
        pidOutput = MathUtil.clamp(m_balancePID.calculate(getRoll(), 0), -0.4, 0.4);
        
        SmartDashboard.putNumber("Balance PID", pidOutput);
        drive(new Translation2d(pidOutput, 0), 0.0, false, true, false);
    }

    public SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        PIDController thetaController = new PIDController(1.2, 0, 0);
        PIDController xController = new PIDController(1.0, 0, 0);
        PIDController yController = new PIDController(1.0, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                    resetOdometry(traj.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 traj, 
                 this::getPose, // Pose supplier
                 SwerveConstants.SWERVE_DRIVE_KINEMATICS, // SwerveDriveKinematics
                 xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 yController, // Y controller (usually the same values as X controller)
                 thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setModuleStates,  // Module states consumer
                 false, //Automatic mirroring
                 this // Requires this drive subsystem
             ) 
             .andThen(() -> stopDrive())
         );
     }

}