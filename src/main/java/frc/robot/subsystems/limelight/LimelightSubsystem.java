// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  public static HttpCamera m_limelight;

  public LimelightSubsystem() {
    m_limelight = new HttpCamera("limelight", "http://limelight.local:5809/stream.mjpg");
    m_limelight.setResolution(320, 240);
    m_limelight.setFPS(90);
    CameraServer.addCamera(m_limelight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    SmartDashboard.putData(SendableCameraWrapper.wrap(m_limelight));
    SmartDashboard.putNumber("X offset", getX());
    SmartDashboard.putNumber("Y offset", getY());
  }
  
  public double getX(){
    return table.getEntry("tx").getDouble(0.0);
  }
  public double getY(){
    return table.getEntry("ty").getDouble(0.0);
  }

  public double getArea(){
    return table.getEntry("ta").getDouble(0.0);
  }

  public double getSkew(){
    return table.getEntry("ts").getDouble(0.0);
  }

  public double getShortLength(){
    return table.getEntry("tshort").getDouble(0.0);
  }

  public double getLongLength(){
    return table.getEntry("tlong").getDouble(0.0);
  }

  public double getHorizantalLength(){
    return table.getEntry("thor").getDouble(0.0);
  }

  public double getVerticalLength(){
    return table.getEntry("tvert").getDouble(0.0);
  }

  public Pose2d getCamTran(){
    double[] camtran = table.getEntry("camtran").getDoubleArray(new double[]{});
    double x = camtran[0];
    double y = camtran[1];
    double yaw = camtran[4];
    return new Pose2d(new Translation2d(x,y), new Rotation2d(yaw));
  }

  public double getDistanceFromTarget(double targetHeightInches){
    double angleToGoalDegrees = LimelightConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + getY();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    //calculate distance
    return (targetHeightInches - LimelightConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(angleToGoalRadians);
  }

  public boolean hasTarget(){
    return table.getEntry("tv").getDouble(0.0) == 1;
  }

  /**
   * @param piplineNumber driver = 0, aprilTags = 1, retroreflective = 2
   */
  public void setPipeline(int pipelineNumber){
    Number numObj = (Number)pipelineNumber;
    table.getEntry("pipeline").setNumber(numObj);
  }

  public boolean inVisionMode(){
    return table.getEntry("pipeline").getDouble(0.0) != 0.0;
  }
}

