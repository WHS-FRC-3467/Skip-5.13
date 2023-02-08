// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.arm.Setpoint;
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
    //Gold Robot
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
    // Alpha Bot

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
    public static final int WRIST_CHANNEL = 0;
    public static final int CLAW_CHANNEL = 1;
  }


  public static final class SwerveConstants{

    public static final double DRIVE_DEADBAND = 0.2;

    //Gold Bot
    //Mod 0
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 109.1;
    //Mod 1
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 253.9;
    //Mod 2
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 111.5; 
    //Mod 3
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 27.2;

    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    //Tailgate (For Testing)
    // public static final COTSFalconSwerveConstants chosenModule =  
    //     COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

    //2023 Robot
    public static final COTSFalconSwerveConstants CHOSEN_MODULE =  
    COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */

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
    public static final Gains GAINS_ANGLE_MOTOR =new Gains(CHOSEN_MODULE.angleKP, CHOSEN_MODULE.angleKI, CHOSEN_MODULE.angleKD, CHOSEN_MODULE.angleKF, 0.0);
    
    /* Drive Motor PID Values */
    public static final Gains GAINS_DRIVE_MOTOR = new Gains(0.05, 0.0, 0.0, 0.0, 0);

    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double DRIVE_KS = (0.32 / 12); 
    public static final double DRIVE_KV = (1.51 / 12);
    public static final double DRIVE_KA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double MAX_SPEED = 1.5;

    /** Radians per Second */
    public static final double MAX_ANGULAR_VELOCITY = 3.0;

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

    public static final Gains GAINS_ANGLE_SNAP = new Gains(0.02, 0.0, 0.0, 0.0, 50);
    
    public static final Gains GAINS_BALANCE = new Gains(0.0,0.0,0.0,0.0, 50);

    public static final double SNAP_TOLLERANCE = 2.0;
    public static final double BALANCE_TOLLERANCE = 0.5;
  }

  public static final class ArmConstants{

    public static final double VERTICAL_ANGLE_UPPER = 171.4;
    public static final double VERTICAL_ANGLE_LOWER = 176.0;

    public static final double LOWER_ANGLE_OFFSET = 180 - VERTICAL_ANGLE_LOWER;
    public static final double UPPER_ANGLE_OFFSET = 180 - VERTICAL_ANGLE_UPPER;

    public static final  Gains GAINS_UPPER_JOINT = new Gains(0.02, 0.0, 0.00, 0.0, 50);

    public static final  Gains GAINS_LOWER_JOINT  = new Gains(0.02, 0.0, 0.00, 0.00, 50);


    public static final double kSUpper = 0.04;
    public static final double kGUpper = 0.25;
    public static final double kVUpper = 0.06;
    public static final double kAUpper = 0.0;

    public static final double kSLower = 0.04;
    public static final double kGLower = 0.25;
    public static final double kVLower = 0.06;
    public static final double kALower = 0.0;

    //JointConfig for DJArmFeedForwards
    public static final double UPPER_LENGTH = 1.07;
    public static final double UPPER_MOI = 0.4;
    public static final double UPPER_CGRADIUS = 1.0;
    public static final double UPPER_MASS = 5.0;
    public static final DCMotor UPPER_MOTOR = DCMotor.getFalcon500(1).withReduction(200);

    public static final double LOWER_LENGTH = 0.7874;
    public static final double LOWER_MOI = 0.4;
    public static final double LOWER_CGRADIUS = 1.0;
    public static final double LOWER_MASS = 5.0;
    public static final DCMotor LOWER_MOTOR = DCMotor.getFalcon500(1).withReduction(200);


    //Max sensor velocity per 100 ms
    //Max RPM 6380 * 2:1 gearing * 4096 ticks *1min/60000ms * 100ms
    public static final int MAX_SENSOR_VEL = 86398;

    /* Motor neutral dead-band : Range 0.001 -> 0.25 */
	  public static final double NEUTRAL_DEADBAND = 0.05;

    public static final double NOMINAL_OUTPUT_FORWARD = 0;
    public static final double NOMINAL_OUTPUT_REVERSE = 0;
    public static final double PEAK_OUTPUT_FORWARD = 0.5;
    public static final double PEAK_OUTPUT_REVERSE = -0.5;

    public static final int FORWARD_SOFT_LIMIT_UPPER = 3300;
    public static final int REVERSE_SOFT_LIMIT_UPPER = 500;
    
    public static final int FORWARD_SOFT_LIMIT_LOWER = 3400;
    public static final int REVERSE_SOFT_LIMIT_LOWER = 1000;
    /**
     * Set to zero to skip waiting for confirmation.
     * Set to nonzero to wait and report to DS if action fails.
    */
	  public final static int TIMEOUT = 10;

    // Motion Magic constants
    public static final double LOWER_CRUISE = 70.0;
    public static final double LOWER_ACCELERATION = 120.0;

    public static final double UPPER_CRUISE = 70.0;
    public static final double UPPER_ACCELERATION = 120.0;

    public static final double DUTY_CYCLE_MIN = 1.0/1025.0;
    public static final double DUTY_CYCLE_MAX = 1024.0/1025.0;
    public static final int FREQUENCY = 976;
    public static final double PERIOD = 1025;

    public static final double ENCODER_DISTANCE_PER_PULSE =  (2.0 * Math.PI / 8192);
  }

  public static final class ArmSetpoints{
    public static final Setpoint TEST_SETPOINT_HIGHER = new Setpoint(191, 35, true, 191, 35, true);
    public static final Setpoint TEST_SETPOINT_LOWER = new Setpoint(164, 65, true, 164, 65, true);

    public static final Setpoint STOWED = new Setpoint(180, 16, false, 180, 16, false);
    public static final Setpoint FLOOR = new Setpoint(243, 61, true, 243, 62, true);
    public static final Setpoint MID_NODE = new Setpoint(189, 85, false, 192, 69, false);
    public static final Setpoint MID_NODE_PLACED = new Setpoint(197, 69, false, 192, 69, false);
    public static final Setpoint TOP_NODE = new Setpoint(225, 149, false, 213, 111, false);
    public static final Setpoint TOP_NODE_PLACED = new Setpoint(234, 149, false, 213, 111, false);
    public static final Setpoint SUBSTATION = new Setpoint(148, 54, false, 137, 47, false);

    public static final double INTERMEDIATE_LOWER_POSITION = 90;
  }

  public static final class LimelightConstants{
    public static final Gains GAINS_VISION_X = new Gains(0.0, 0.0, 0.0, 0.0, 50);
    public static final Gains GAINS_VISION_Y = new Gains(0.0, 0.0, 0.0, 0.0, 50);

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
