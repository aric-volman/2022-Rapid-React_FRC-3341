// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.BallHandler;


public class EncoderShoot extends CommandBase {

  private BallHandler ballHandler;

  private double minimumRollerPower = 1.0; // We also need to test for this if we have the time
  private double rollerpower = 1.0;
  private boolean isFlywheelAtSpeed;
  private double rollBackTime = 0.5; // Something that needs to be tested
  private double cargoIsLaunchedTime = 4.0; // Arguably the most important timer
  private double velocity;

  Timer cargoTimer = new Timer();

  /** Creates a new EncoderShoot. */
  public EncoderShoot(double velocity, BallHandler bh) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ballHandler = bh;
    this.velocity = velocity;
    cargoTimer.start();
    addRequirements(ballHandler); // If you put this before assignment, bad stuff happens
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFlywheelAtSpeed = false;
    ballHandler.resetFlywheelEncoders();
    cargoTimer.reset();
    ballHandler.setRollerPower(-minimumRollerPower);
    Timer.delay(rollBackTime);
    ballHandler.setRollerPower(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Comment these out if you feel PID is needed
    // ballHandler.setFlywheelConstantVelocity(velocity);
    // if (ballHandler.flywheelWithinErrorMargin()) {
    
    ballHandler.setFlywheelPower(1.0);
    if (ballHandler.getAverageRPM() >= 1800 && !isFlywheelAtSpeed) { // RPM based
      ballHandler.setRollerPower(rollerpower);
      isFlywheelAtSpeed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballHandler.setFlywheelPower(0.0);
    ballHandler.setRollerPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (cargoTimer.get() >= cargoIsLaunchedTime);
    
  }
}
