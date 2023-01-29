// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
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

  public static final boolean tuningMode = true;

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

    public static final int CLAW_MOTOR = 16;

    public static final int LED_CANDLE = 17;
  }
  
  public static final class DIOConstants{
    public static final int LOWER_ENCODER_ARM = 1;
    public static final int UPPER_ENCODER_ARM = 0;

  }
  public static final class PHConstants{
    public static final int CLAW_JOINT_CHANNEL = 0;
  }


  public static final class SwerveConstants{

    public static final double DRIVE_DEADBAND = 0.2;


    //Mod 0
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 43.9;
    //Mod 1
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 17.75;
    //Mod 2
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 263.1; 
    //Mod 3
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 358.7;

    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    //Tailgate (For Testing)
    // public static final COTSFalconSwerveConstants chosenModule =  
    //     COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

    //2023 Robot
    public static final COTSFalconSwerveConstants CHOSEN_MODULE =  
    COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    //Tailgate (For testing)
    // public static final double trackWidth = Units.inchesToMeters(21.0); 
    // public static final double wheelBase = Units.inchesToMeters(21.0); 
    //2023 Robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(18.75); 
    public static final double WHEEL_BASE = Units.inchesToMeters(18.75); 

    public static final double WHEEL_CIRCUMFRENCE = CHOSEN_MODULE.wheelCircumference;

    
    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /* Module Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
    public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

    /* Motor Inverts */
    public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
    public static final boolean DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean CANCODER_INVERT = CHOSEN_MODULE.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
    public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Angle Motor PID Values */
    public static final Gains GAINS_ANGLE_MOTOR =new Gains(CHOSEN_MODULE.angleKP, CHOSEN_MODULE.angleKI, CHOSEN_MODULE.angleKD, CHOSEN_MODULE.angleKF, 0.0, 0.0);
    
    /* Drive Motor PID Values */
    public static final Gains GAINS_DRIVE_MOTOR = new Gains(0.05, 0.0, 0.0, 0.0, 0, 0);

    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double DRIVE_KS = (0.32 / 12); 
    public static final double DRIVE_KV = (1.51 / 12);
    public static final double DRIVE_KA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double MAX_SPEED = 0.25;

    /** Radians per Second */
    public static final double MAX_ANGULAR_VELOCITY = 0.5;

    /* Neutral Modes */
    public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

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
        public static final int canCoderID = CanConstants.BACK_LEFT_MODULE_STEER_ENCODER;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(BACK_LEFT_MODULE_STEER_OFFSET);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { 
        public static final int driveMotorID = CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
        public static final int angleMotorID = CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR;
        public static final int canCoderID = CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(BACK_RIGHT_MODULE_STEER_OFFSET);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public static final Gains GAINS_ANGLE_SNAP = new Gains(0.0,0.0,0.0,0.0, 50, 0.0);
    public static final Gains GAINS_BALANCE = new Gains(0.0,0.0,0.0,0.0, 50, 0.0);

    public static final double SNAP_TOLLERANCE = 2.0;
    public static final double BALANCE_TOLLERANCE = 0.5;
  }

  public static final class ArmConstants{
    /**
     * kF = minimum percent output to break friction
     */
    public static final  Gains GAINS_UPPER_JOINT = new Gains(0.03, 0.004, 0.00, 0.0, 50, 1.00);

    public static final  Gains GAINS_LOWER_JOINT  = new Gains(0.04, 0.004, 0.00, 0.00, 50, 1.00);

    public static final double kGUpper = 0.38;
    public static final double kVUpper = 0.06;
    public static final double kAUpper = 0.0;

    //Max sensor velocity per 100 ms
    //Max RPM 6380 * 2:1 gearing * 4096 ticks *1min/60000ms * 100ms
    public static final int MAX_SENSOR_VEL = 86398;

    /* Motor neutral dead-band : Range 0.001 -> 0.25 */
	  public static final double NEUTRAL_DEADBAND = 0.05;

    public static final double NOMINAL_OUTPUT_FORWARD = 0;
    public static final double NOMINAL_OUTPUT_REVERSE = 0;
    public static final double PEAK_OUTPUT_FORWARD = 0.3;
    public static final double PEAK_OUTPUT_REVERSE = -0.3;

    public static final int REVERSE_SOFT_LIMIT_UPPER = 100;
    public static final int FORWARD_SOFT_LIMIT_UPPER = 3500;

    public static final int REVERSE_SOFT_LIMIT_LOWER = 1800;
    public static final int FORWARD_SOFT_LIMIT_LOWER = 2800;
    /**
     * Set to zero to skip waiting for confirmation.
     * Set to nonzero to wait and report to DS if action fails.
    */
	  public final static int TIMEOUT = 10;

    public static final double TOLERANCE_UPPER = 2.0;
    public static final double TOLERANCE_LOWER = 2.0;

    // Motion Magic constants
    public static final double LOWER_CRUISE = 0.5;
    public static final double LOWER_ACCELERATION = 0.5;

    public static final double UPPER_CRUISE = 0.1;
    public static final double UPPER_ACCELERATION = 0.1;

    public static final double DUTY_CYCLE_MIN = 1.0/1025.0;
    public static final double DUTY_CYCLE_MAX = 1024.0/1025.0;
    public static final int FREQUENCY = 976;
    public static final double PERIOD = 1025;

    public static final double ENCODER_DISTANCE_PER_PULSE =  (2.0 * Math.PI / 8192);

    //set points
    public static final double UPPER_HORIZANTAL_FORWARD = 115;
    public static final double UPPER_VERTICAL = 176;
    public static final double UPPER_HORIZANTAL_BACKWARD = 291;

    public static final double LOWER_HORIZANTAL_FORWARD = 264;
    public static final double LOWER_VERTICAL = 175;
    public static final double LOWER_HORIZANTAL_BACKWARD = 82;
  }

  public static final class ArmSetpoints{
    //Setpoints

    public static final double STOWED_UPPER = 308;
    public static final double STOWED_LOWER = 186;

    public static final double CONENODE_TOP_UPPER = 0;
    public static final double CONENODE_TOP_LOWER = 0;

    public static final double CONENODE_MID_UPPER = 0;
    public static final double CONENODE_MID_LOWER = 0;

    public static final double CUBENODE_TOP_UPPER = 207;
    public static final double CUBENODE_TOP_LOWER = 153;

    public static final double CUBENODE_MID_UPPER = 250;
    public static final double CUBENODE_MID_LOWER = 179;

    public static final double DOUBLE_SUBSTATION_CONE_UPPER = 251;
    public static final double DOUBLE_SUBSTATION_CONE_LOWER = 194;

    public static final double DOUBLE_SUBSTATION_CUBE_UPPER = 265;
    public static final double DOUBLE_SUBSTATION_CUBE_LOWER = 223;

    public static final double FLOOR_CONE_UPPER = 0;
    public static final double FLOOR_CONE_LOWER= 0;

    public static final double FLOOR_CUBE_UPPER = 279;
    public static final double FLOOR_CUBE_LOWER= 163;
  }

  public static final class LimelightConstants{
    public static final Gains GAINS_VISION_X = new Gains(0.0, 0.0, 0.0, 0.0, 50, 1.0);
    public static final Gains GAINS_VISION_Y = new Gains(0.0, 0.0, 0.0, 0.0, 50, 1.0);

    public static final double VISION_VEL_TOLLERANCE = 0.5;
    public static final double VISION_POS_TOLLERANCE = 0.5;

    // how many degrees back is your limelight rotated from perfectly vertical?
    public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 25.0;

    // distance from the center of the Limelight lens to the floor
    public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 20.0;

    public static final double SETPOINT_DIS_FROM_MID_CONE = 24;
    public static final double SETPOINT_DIS_FROM_TOP_CONE = 40;

    public static final double SETPOINT_DIS_FROM_GRID_APRIL = 14.062222;
    public static final double SETPOINT_DIS_FROM_SUBSTATION_APRIL = 5;

    public static final double ALIGNED_CONE_X = 0.0;
    public static final double ALIGNED_GRID_APRIL_X = 0.0;
    public static final double ALIGNED_SUBSTATION_APRIL_X = 0.0;


    // height of vision tape center in inches
    public static final double HEIGHT_CONE_NODE_TAP = 24.125;
    public static final double HEIGHT_GRID_APRIL = 18.25;
    public static final double HEIGHT_SUBSTATION_APRIL = 27.375;
  }
}
