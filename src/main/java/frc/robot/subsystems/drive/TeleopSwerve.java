package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private DriveSubsystem m_drive;    
    private DoubleSupplier m_xTranslation, m_yTranslation, m_rotation;
    private Boolean m_robotCentric;

    public TeleopSwerve(DriveSubsystem s_Swerve, DoubleSupplier xTranslation, DoubleSupplier yTranslation, DoubleSupplier rotation, Boolean robotCentric) {
        m_drive = s_Swerve;
        m_xTranslation = xTranslation;
        m_yTranslation = yTranslation;
        m_rotation = rotation;
        m_robotCentric = robotCentric;
        addRequirements(s_Swerve);

        ;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double xVal = MathUtil.applyDeadband(m_xTranslation.getAsDouble(), DriveConstants.kDeadBand);
        double yVal = MathUtil.applyDeadband(m_yTranslation.getAsDouble(), DriveConstants.kDeadBand);
        double rotationVal = MathUtil.applyDeadband(m_rotation.getAsDouble(), DriveConstants.kDeadBand);

        /* Drive */
        m_drive.drive(
            new Translation2d(xVal, yVal).times(Constants.DriveConstants.maxSpeed), 
            rotationVal * Constants.DriveConstants.maxAngularVelocity, 
            m_robotCentric, 
            true
        );
    }
}