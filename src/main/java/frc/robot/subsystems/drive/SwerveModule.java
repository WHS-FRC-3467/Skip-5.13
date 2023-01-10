package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.util.CTREModuleState;
import frc.robot.util.Conversions;
import frc.robot.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driverMotor;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.driveKS, Constants.DriveConstants.driveKV, Constants.DriveConstants.driveKA);


    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        m_driverMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;


        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.DriveConstants.angleEnableCurrentLimit, 
            Constants.DriveConstants.angleContinuousCurrentLimit, 
            Constants.DriveConstants.anglePeakCurrentLimit, 
            Constants.DriveConstants.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.DriveConstants.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.DriveConstants.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.DriveConstants.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.DriveConstants.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.DriveConstants.driveEnableCurrentLimit, 
            Constants.DriveConstants.driveContinuousCurrentLimit, 
            Constants.DriveConstants.drivePeakCurrentLimit, 
            Constants.DriveConstants.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.DriveConstants.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.DriveConstants.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.DriveConstants.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.DriveConstants.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.DriveConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.DriveConstants.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.DriveConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.DriveConstants.maxSpeed;
            m_driverMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.DriveConstants.wheelCircumference, Constants.DriveConstants.driveGearRatio);
            m_driverMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DriveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.DriveConstants.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), Constants.DriveConstants.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.DriveConstants.angleGearRatio);
        m_angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        m_angleMotor.configFactoryDefault();
        m_angleMotor.configAllSettings(swerveAngleFXConfig);
        m_angleMotor.setInverted(Constants.DriveConstants.angleMotorInvert);
        m_angleMotor.setNeutralMode(Constants.DriveConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        m_driverMotor.configFactoryDefault();
        m_driverMotor.configAllSettings(swerveDriveFXConfig);
        m_driverMotor.setInverted(Constants.DriveConstants.driveMotorInvert);
        m_driverMotor.setNeutralMode(Constants.DriveConstants.driveNeutralMode);
        m_driverMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(m_driverMotor.getSelectedSensorVelocity(), Constants.DriveConstants.wheelCircumference, Constants.DriveConstants.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(m_driverMotor.getSelectedSensorPosition(), Constants.DriveConstants.wheelCircumference, Constants.DriveConstants.driveGearRatio), 
            getAngle()
        );
    }

    
}