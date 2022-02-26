// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.BallHandler;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetAnglePID extends PIDCommand {
  private BallHandler bh;
  private Timer timeout = new Timer();
  // Adjust this when it's appropriate
  private double notThereYetTime = 3.0;
  private double tolerance = 1.5;
  /** Creates a new SetAnglePID. 
   * 
   * 
  */
  public SetAnglePID(double angle, BallHandler ballHandler) {
    super(
        // The controller that the command will use
        new PIDController(Constants.pivotPIDConsts.pidP, Constants.pivotPIDConsts.pidI, Constants.pivotPIDConsts.pidD),
        // This should return the measurement
        ballHandler::getPivotPosition,
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output - sign is specific for the hood
        output -> ballHandler.setPivotPower(-1.0*output), 
        ballHandler);
    this.bh = ballHandler;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(ballHandler);
    getController().setTolerance(tolerance);
  }

  @Override
  public void initialize() {
    timeout.start();
    timeout.reset();
  }

  // Returns true when the command should end.
  @Override
  public void end(boolean interrupted) {
    bh.setPivotPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return (getController().atSetpoint() | timeout.get() >= notThereYetTime);
  }
}
