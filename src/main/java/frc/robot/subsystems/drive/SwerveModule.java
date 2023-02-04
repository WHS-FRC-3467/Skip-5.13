package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Conversions;
import frc.robot.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private CTREConfigs m_configs = new CTREConfigs();

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, "drive");
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "drive");
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "drive");
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean isTeleop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop, isTeleop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop, boolean isTeleop){
        if(isOpenLoop && isTeleop){
            double speed = Math.copySign(Math.pow(desiredState.speedMetersPerSecond, 2), desiredState.speedMetersPerSecond);
            double percentOutput = speed / SwerveConstants.MAX_SPEED;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else if (!isOpenLoop && !isTeleop) {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.WHEEL_CIRCUMFRENCE, SwerveConstants.DRIVE_GEAR_RATIO);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        else if (!isOpenLoop && isTeleop){
            double speed = Math.copySign(Math.pow(desiredState.speedMetersPerSecond, 2), desiredState.speedMetersPerSecond);
            double velocity = Conversions.MPSToFalcon(speed, SwerveConstants.WHEEL_CIRCUMFRENCE, SwerveConstants.DRIVE_GEAR_RATIO);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        else if (isOpenLoop && !isTeleop){
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.ANGLE_GEAR_RATIO));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveConstants.ANGLE_GEAR_RATIO));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.ANGLE_GEAR_RATIO);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(m_configs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(m_configs.swerveAngleFXConfig);
        mAngleMotor.setInverted(SwerveConstants.ANGLE_MOTOR_INVERT);
        mAngleMotor.setNeutralMode(SwerveConstants.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(m_configs.swerveDriveFXConfig);
        mDriveMotor.setInverted(SwerveConstants.DRIVE_MOTOR_INVERT);
        mDriveMotor.setNeutralMode(SwerveConstants.DRIVE_NEUTRAL_MODE);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), SwerveConstants.WHEEL_CIRCUMFRENCE, SwerveConstants.DRIVE_GEAR_RATIO), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), SwerveConstants.WHEEL_CIRCUMFRENCE, SwerveConstants.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }

    
}