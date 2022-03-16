// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RotatePID extends CommandBase {
  /** Creates a new RotatePID. */
  private double angle;
  private int motorNum;
  private PIDController pid;
  private double kp = 0.01;
  private double ki = 0.0005;
  private double kd = 0.0;
  private double errorAccum;
  private Timer time;
  private double prevTime;


  public RotatePID(int motorNum, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
   // addRequirements(RobotContainer.getArm());
    this.angle = angle;
    this.motorNum = motorNum;
    time = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(kp, ki, kd);
    pid.setSetpoint(angle);
    time.reset();
    errorAccum = 0.0;
    prevTime = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Rotate Accum Error2:", errorAccum);
    
    double currTime = Timer.getFPGATimestamp();
    double posError = pid.getPositionError();

    double dt = currTime - prevTime;
    prevTime = currTime;

    //pid.calculate(RobotContainer.getArm().getArmPosition(motorNum));
    errorAccum += (posError) * dt;

    double pidPower = (kp * posError) + (ki * errorAccum);

    //RobotContainer.getArm().rotate(1, pidPower);



    //SmartDashboard.putNumber("RotatePIDPow:", pidPower);
    //SmartDashboard.putNumber("Rotate kiPower:", ki * errorAccum);
    SmartDashboard.putNumber("Rotate Accum Error1:", errorAccum);
    //SmartDashboard.putNumber("Rotate Error:", pid.getPositionError());
    //SmartDashboard.putNumber("Rotate kpPower:", kp * pid.getPositionError());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // RobotContainer.getArm().rotatePow(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("isFinished:", Math.abs(pid.getPositionError()) <= 2);
    return Math.abs(pid.getPositionError()) <= 2;
    //return pid.atSetpoint() || RobotContainer.getJoy1().getRawButton(1);
  }
}
