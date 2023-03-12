package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.NodePoints;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private int m_selectedNode;

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
        m_selectedNode = 0;
    }

    @Override
    public void periodic(){
        // if(DriverStation.isDisabled()){
        //     resetModulesToAbsolute();
        // }
        // for(SwerveModule mod : mSwerveMods){
        //     mod.putToTempDashboard();
        // } 

        m_balancePID.setTolerance(SwerveConstants.BALANCE_TOLLERANCE);
        //double pidOutput = m_balancePID.calculate(getRoll(), 0);
        // SmartDashboard.putNumber("Balance PID", pidOutput);
        // SmartDashboard.putNumber("Robot Pitch", getPitch());
        // SmartDashboard.putNumber("Robot Roll", getRoll());

        //SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().name());

        swerveOdometry.update(getYaw(), getModulePositions()); 

        m_field.setRobotPose(swerveOdometry.getPoseMeters());

        SmartDashboard.putNumber("Selected Node", m_selectedNode);

        SmartDashboard.putData("Feild", m_field);
        if(Constants.tuningMode){
            SmartDashboard.putNumber("gyro Yaw", getYaw().getDegrees());

            for(SwerveModule mod : mSwerveMods){
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            } 
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
        SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    swerveOdometry.getPoseMeters().getRotation()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        // SmartDashboard.putNumber("X Translation", translation.getX());
        // SmartDashboard.putNumber("Y Translation", translation.getY());
        // SmartDashboard.putNumber("Rotation Value", rotation);
    }    

    public void stopDrive(){
        drive(new Translation2d(0, 0), 0, false, true);
    }


    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
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
        // swerveOdometry.resetPosition(getYaw(), getModulePositions(), new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    }

    public void setYaw(double yaw){
        m_gyro.setYaw(yaw);
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
    public int getSelectedNodeInt(){
        return m_selectedNode;
    }

    public void setSelectedNode(int node){
        if(node<0){
            node = 0;
        }
        if(node>0){
            node = 8;
        }
        m_selectedNode = node;
    }
    public void addOneToSelectedNode(){
        if(m_selectedNode<8){
            m_selectedNode++;  
        }
    }

    public void subtractOneToSelectedNode(){
        if(m_selectedNode>1){
            m_selectedNode--;
        }
    }
    public PathPlannerTrajectory pathToScore(){
        return  PathPlanner.generatePath(new PathConstraints(1.0, 1.0), 
        new PathPoint(swerveOdometry.getPoseMeters().getTranslation(), 
                      swerveOdometry.getPoseMeters().getRotation(),
                      swerveOdometry.getPoseMeters().getRotation()),
                      getSelectedNode());
    }
    public SequentialCommandGroup followPathToScoreGroup() {
        PIDController thetaController = new PIDController(2.0, 0, 0);
        PIDController xController = new PIDController(1.3, 0, 0);
        PIDController yController = new PIDController(1.2, 0, 0);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SequentialCommandGroup(
             
             new PPSwerveControllerCommand(
                 pathToScore(), 
                 this::getPose, // Pose supplier
                 SwerveConstants.SWERVE_DRIVE_KINEMATICS, // SwerveDriveKinematics
                 xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 yController, // Y controller (usually the same values as X controller)
                 thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setModuleStates,  // Module states consumer
                 true, //Automatic mirroring
                 this // Requires this drive subsystem
             ) 
             .andThen(() -> stopDrive())
         );
     }
    public PathPoint getSelectedNode(){
        if(DriverStation.getAlliance() == Alliance.Red){
            if(m_selectedNode == 0){
                return NodePoints.ZERO_RED;
            }
            else if(m_selectedNode == 1){
                return NodePoints.ONE_RED;
            }
            else if(m_selectedNode == 2){
                return NodePoints.TWO_RED;
            }
            else if(m_selectedNode == 3){
                return NodePoints.THREE_RED;
            }
            else if(m_selectedNode == 4){
                return NodePoints.FOUR_RED;
            }
            else if(m_selectedNode == 5){
                return NodePoints.FIVE_RED;
            }
            else if(m_selectedNode == 6){
                return NodePoints.SIX_RED;
            }
            else if(m_selectedNode == 7){
                return NodePoints.SEVEN_RED;
            }
            else if(m_selectedNode == 8){
                return NodePoints.EIGHT_RED;
            }
            else{
                return null;
            }
        }
        else if(DriverStation.getAlliance() == Alliance.Blue){
            if(m_selectedNode == 0){
                return NodePoints.ZERO_BLUE;
            }
            else if(m_selectedNode == 1){
                return NodePoints.ONE_BLUE;
            }
            else if(m_selectedNode == 2){
                return NodePoints.TWO_BLUE;
            }
            else if(m_selectedNode == 3){
                return NodePoints.THREE_BLUE;
            }
            else if(m_selectedNode == 4){
                return NodePoints.FOUR_BLUE;
            }
            else if(m_selectedNode == 5){
                return NodePoints.FIVE_BLUE;
            }
            else if(m_selectedNode == 6){
                return NodePoints.SIX_BLUE;
            }
            else if(m_selectedNode == 7){
                return NodePoints.SEVEN_BLUE;
            }
            else if(m_selectedNode == 8){
                return NodePoints.EIGHT_BLUE;
            }
            else{
                return null;
            }
        }
        else{
            return null;
        }
    }

    public void AutoBalance(){
        m_balancePID.setTolerance(SwerveConstants.BALANCE_TOLLERANCE);
        double pidOutput;
        pidOutput = MathUtil.clamp(m_balancePID.calculate(getRoll(), 0), -1.0, 1.0);
        // if(Constants.tuningMode){
        //     SmartDashboard.putNumber("Balance PID", pidOutput);
        // }
        drive(new Translation2d(-pidOutput, 0), 0.0, false, true);
    }

    public SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory path1, boolean isFirstPath) {
        PIDController thetaController = new PIDController(2.0, 0, 0);
        PIDController xController = new PIDController(1.3, 0, 0);
        PIDController yController = new PIDController(1.2, 0, 0);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                    PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(path1, DriverStation.getAlliance());
                    resetOdometry(transformed.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 path1, 
                 this::getPose, // Pose supplier
                 SwerveConstants.SWERVE_DRIVE_KINEMATICS, // SwerveDriveKinematics
                 xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 yController, // Y controller (usually the same values as X controller)
                 thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setModuleStates,  // Module states consumer
                 true, //Automatic mirroring
                 this // Requires this drive subsystem
             ) 
             .andThen(() -> stopDrive())
         );
     }
}