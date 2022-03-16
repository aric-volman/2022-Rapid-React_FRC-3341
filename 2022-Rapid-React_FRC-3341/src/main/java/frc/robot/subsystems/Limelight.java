// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//http://10.33.41.11:5800
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight-drswish");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  public void update(){
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  public void setRedPipeline(){
    table.getEntry("pipeline").setNumber(0);
  }

  public void setBluePipeline(){
    table.getEntry("pipeline").setNumber(1);
  }

  public void setTapePipeline(){
    table.getEntry("pipeline").setNumber(2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    //update();
    //setBluePipeline();
    //setRedPipeline();
    //setTapePipeline();
    
  }
}
