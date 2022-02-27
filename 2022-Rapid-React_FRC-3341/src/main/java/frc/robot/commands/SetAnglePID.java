// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.BallHandler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetAnglePID extends CommandBase {
  private BallHandler ballHandler;
  private Timer timeout = new Timer();
  // Adjust this when it's appropriate
  private double notThereYetTime = 3.0; // We really need to adjust this
  private double angle = 0.0;
  /** Creates a new SetAnglePID. 
   * 
   * 
  */
  public SetAnglePID(double angle, BallHandler bh) {
    this.ballHandler = bh;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(ballHandler);
  }

  @Override
  public void initialize() {
    timeout.start();
    timeout.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballHandler.setPivotAngle(angle);
  }

  // Returns true when the command should end.
  @Override
  public void end(boolean interrupted) {
    ballHandler.setPivotPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return (ballHandler.atSetpoint() | timeout.get() >= notThereYetTime);
  }
}
