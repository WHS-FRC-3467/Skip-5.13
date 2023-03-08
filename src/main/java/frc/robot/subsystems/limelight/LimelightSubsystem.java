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
    if(Constants.tuningMode){
      SmartDashboard.putNumber("X offset", getX());
      SmartDashboard.putNumber("Y offset", getY());
      SmartDashboard.putNumber("Target Area", getArea()); 
    }
    
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
  
  public boolean hasTarget(){
    return table.getEntry("tv").getDouble(0.0) == 1;
  }

  public int getID(){
    return (int)table.getEntry("tid").getDouble(0.0);
  }

  public double getLatPip(){
    return table.getEntry("tl").getDouble(0.0)/1000.0;
  }
  public double getLatCap(){
    return table.getEntry("cl").getDouble(0.0)/1000.0;

  }

  public Pose3d getBotPose(){
    double[] pose = table.getEntry("botpose").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }
  public Pose3d getBotPoseRed(){
    double[] pose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }
  public Pose3d getBotPoseBlue(){
    double[] pose = table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }
  public Transform3d getTransform(){
    return new Transform3d(new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)), getBotPose());
  }

  public double getLastEntryTimeStamp(){
    return Timer.getFPGATimestamp() - getLatCap() - getLatPip();
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

