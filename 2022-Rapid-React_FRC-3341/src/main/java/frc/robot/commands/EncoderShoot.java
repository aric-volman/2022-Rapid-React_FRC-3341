// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.BallHandler;


public class EncoderShoot extends CommandBase {

  private BallHandler ballhandler;

  double velocity;
  boolean isFlywheelAtSpeed;

  // placeholder
  double cargoIsLaunchedTime = 4.0;

  // placeholder
  private double rollerpower = 0.15;
  Timer cargoTimer = new Timer();

  /** Creates a new EncoderShoot. */
  public EncoderShoot(double v, BallHandler bh) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ballhandler = bh;
    this.velocity = v;
    cargoTimer.start();
    addRequirements(ballhandler); // If you put this before assignment, bad stuff happens
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFlywheelAtSpeed = false;
    ballhandler.resetFlywheelEncoders();
    cargoTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ballhandler.setFlywheelConstantVelocity(velocity);
    if (ballhandler.flywheelWithinErrorMargin()) {
    // if(ballhandler.flywheelWithinErrorMargin() && !isFlywheelAtSpeed) {
      ballhandler.setRollerPower(rollerpower);
      //isFlywheelAtSpeed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballhandler.setFlywheelPower(0.0);
    ballhandler.setRollerPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (cargoTimer.get() >= cargoIsLaunchedTime);
    
  }
}
