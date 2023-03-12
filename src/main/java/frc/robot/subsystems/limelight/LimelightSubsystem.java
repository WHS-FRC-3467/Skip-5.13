// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */  
  NetworkTable tableRear = NetworkTableInstance.getDefault().getTable("limelight-rear");
  NetworkTable tableFront = NetworkTableInstance.getDefault().getTable("limelight-front");

  public static HttpCamera m_limelightRear;
  public static HttpCamera m_limelightFront;

  private boolean m_visionMode;

  public LimelightSubsystem() {
    m_limelightRear = new HttpCamera("RearLL", "http://limelight.rear:5809/stream.mjpg");
    m_limelightRear.setResolution(320, 240);
    m_limelightRear.setFPS(90);
    m_limelightFront = new HttpCamera("FrontLL", "http://limelight.front:5809/stream.mjpg");
    m_limelightFront.setResolution(320, 240);
    m_limelightFront.setFPS(90);

    CameraServer.addCamera(m_limelightRear);
    CameraServer.addCamera(m_limelightFront);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    SmartDashboard.putData(SendableCameraWrapper.wrap(m_limelightRear));
    SmartDashboard.putData(SendableCameraWrapper.wrap(m_limelightFront));
    if(Constants.tuningMode){
      SmartDashboard.putNumber("X offset", getXFront());
      SmartDashboard.putNumber("Y offset", getYFront());
      SmartDashboard.putNumber("Target Area", getAreaFront()); 
      SmartDashboard.putBoolean("Has target", hasTargetFront());
    }
    
  }
  
  public double getXRear(){
    return tableRear.getEntry("tx").getDouble(0.0);
  }

  public double getYRear(){
    return tableRear.getEntry("ty").getDouble(0.0);
  }

  public double getAreaRear(){
    return tableRear.getEntry("ta").getDouble(0.0);
  }


  public boolean hasTargetRear(){
    return tableRear.getEntry("tv").getDouble(0.0) == 1;
  }

  public int getIDRear(){
    return (int)tableRear.getEntry("tid").getDouble(0.0);
  }

  public double getLatPipRear(){
    return tableRear.getEntry("tl").getDouble(0.0)/1000.0;
  }

  public double getLatCapRear(){
    return tableRear.getEntry("cl").getDouble(0.0)/1000.0;
  }

  
  public Pose3d getBotPose(){
    double[] pose = tableRear.getEntry("botpose").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }
  public Pose3d getBotPoseRed(){
    double[] pose = tableRear.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }
  public Pose3d getBotPoseBlue(){
    double[] pose = tableRear.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }

  public Transform3d getTransform(){
    return new Transform3d(new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)), getBotPose());
  }

  public double getLastEntryTimeStamp(){
    return Timer.getFPGATimestamp() - getLatCapRear() - getLatPipRear();
  }

  /**
   * @param piplineNumber driver = 0, aprilTags = 1, retroreflective = 2
   */
  public void setPipelineRear(int pipelineNumber){
    Number numObj = (Number)pipelineNumber;
    tableRear.getEntry("pipeline").setNumber(numObj);
  }

  
  public double getXFront(){
    return tableFront.getEntry("tx").getDouble(0.0);
  }
  public double getYFront(){
    return tableFront.getEntry("ty").getDouble(0.0);
  }
  
  public double getAreaFront(){
    return tableFront.getEntry("ta").getDouble(0.0);
  }
  
  public boolean hasTargetFront(){
    return tableFront.getEntry("tv").getDouble(0.0) == 1;
  }
  
  public int getIDFront(){
    return (int)tableFront.getEntry("tid").getDouble(0.0);
  }
  
  /**
   * @param piplineNumber 0 = april tags
   */
  public void setPipelineFront(int pipelineNumber){
    Number numObj = (Number)pipelineNumber;
    tableFront.getEntry("pipeline").setNumber(numObj);
  }


  public boolean inVisionMode(){
    return m_visionMode;
  }
  public void setVisionModeOn(){
    m_visionMode = true;
  }
  public void setVisionModeOff(){
    m_visionMode = false;
  }
}
