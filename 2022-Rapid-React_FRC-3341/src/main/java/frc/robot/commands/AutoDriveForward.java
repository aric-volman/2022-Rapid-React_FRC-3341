// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

/** An example command that uses an example subsystem. */
public class AutoDriveForward extends CommandBase 
{
  private final DriveTrain _DriveTrain;

  private double distance;
  private double speed;
  private double error;
  private double kP = 0.7; //test constant later
  private int direction; //0 for forward, 1 for backward

  /**
  @param dist - Distance in meters
  */

  public AutoDriveForward(DriveTrain dt, double dist) 
  {
    distance = Math.abs(dist);
    _DriveTrain = dt;
    addRequirements(_DriveTrain);
    if(dist < 0)
    {
      direction = 1;
    }
    if(dist > 0)
    {
      direction = 0;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    _DriveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Ticks", _DriveTrain.getTicks());
    error = distance - Math.abs(_DriveTrain.getPosition());
    error = (error / distance)*2;
    speed = error * kP;

    if(speed > .5)
    {
      speed = .5;
    }

    if(speed < .2)
    {
      speed = .2;
    }

    SmartDashboard.putNumber("Current Speed", speed);
    SmartDashboard.putNumber("Current Distance", _DriveTrain.getPosition());
    if(direction == 1)
    {
      _DriveTrain.tankDrive(-speed,-speed);
    }
    if(direction == 0)
    {
      _DriveTrain.tankDrive(speed,speed);
    }

    System.out.println(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    _DriveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return Math.abs(_DriveTrain.getPosition()) >= distance;
  }
}
