// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.util.COTSFalconSwerveConstants;
import frc.robot.util.Gains;
import frc.robot.util.SwerveModuleConstants;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean tuningMode = false;

  public static final class CanConstants{
    //drivebase CAN IDs 
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; 

    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; 

    public static final int PIGEON2 = 13;

    public static final int LOWER_JOINT_MOTOR = 14;
    public static final int UPPER_JOINT_MOTOR = 15;

  }
  
  public static final class DriveConstants{
    public static final double kDeadBand = 0.2;
    public static final boolean PRACTICE = true;

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 105.7;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 289.2;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 140.1; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 98.3;

    // Limelight auto aim X-axis target tolerance. This is the number of degrees
    // from perfect center that the robot will consider the BasicLimelightAim
    // command "finished".
    public static final double LIMELIGHT_X_TOLERANCE = 1.0;

    // Maximum Limelight auto-aim rotation velocity in radians per second.
    public static final double LIMELIGHT_X_VELOCITY_LIMIT = 0.5;

    // Limelight auto-aim X-axis P-gain.
    public static final double LIMELIGHT_X_P = 10.0;

        
    public static final double precisionSpeed = 0.25;

    //meters per second
    public static final double SimpleAutoVelocity = 1.0;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =  
        COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = 0.5334; 
    public static final double wheelBase = 0.5334; 
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    
    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 3.0;
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0;

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { 
        public static final int driveMotorID = CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
        public static final int angleMotorID = CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR;
        public static final int canCoderID = CanConstants.FRONT_LEFT_MODULE_STEER_ENCODER;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(FRONT_LEFT_MODULE_STEER_OFFSET);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
        public static final int angleMotorID = CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR;
        public static final int canCoderID = CanConstants.FRONT_RIGHT_MODULE_STEER_ENCODER;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(FRONT_RIGHT_MODULE_STEER_OFFSET);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 { 
        public static final int driveMotorID = CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR;
        public static final int angleMotorID = CanConstants.BACK_LEFT_MODULE_STEER_MOTOR;
        public static final int canCoderID = CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(BACK_RIGHT_MODULE_STEER_OFFSET);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { 
        public static final int driveMotorID = CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR;
        public static final int angleMotorID = CanConstants.BACK_LEFT_MODULE_STEER_MOTOR;
        public static final int canCoderID = CanConstants.BACK_LEFT_MODULE_STEER_ENCODER;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(BACK_LEFT_MODULE_STEER_OFFSET);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class ArmConstants{
    public static final  Gains kGainsUpperJoint = new Gains( 0.5, 0.0, 0.01, .05, 100, 1.00 );
    public static final  Gains kGainsLowerJoint  = new Gains( 0.2, 0.0, 0.0, 0.0, 200, 1.00 );

    /* Motor neutral dead-band : Range 0.001 -> 0.25 */
	  public static final double kNeutralDeadband = 0.001;

    public static final double kNominalOutputForward = 1.0;
    public static final double kNominalOutputReverse = -1.0;
    public static final double kPeakOutputForward = 1.0;
    public static final double kPeakOutputReverse = -1.0;

	  /* Current Limit for arm calibration */
    public static final double kCalibCurrentLimit = 10.0;


    /**
     * Set to zero to skip waiting for confirmation.
     * Set to nonzero to wait and report to DS if action fails.
    */
	  public final static int kTimeoutMs = 20;

    // Motion Magic constants
    public static final int kMotionCruiseVelocityLower = 25000;
    public static final int kMotionAccelerationLower = 35000;
    public static final int kCurveSmoothingLower = 0;  /* Valid values: 0 -> 8 */
    public static final int kToleranceLower = 500;

    public static final int kMotionCruiseVelocityUpper = 25000;
    public static final int kMotionAccelerationUpper = 35000;
    public static final int kCurveSmoothingUpper = 0;  /* Valid values: 0 -> 8 */
    public static final int kToleranceUpper = 500;


  }
}
