// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.BallHandler;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetAnglePID extends PIDCommand {
  private BallHandler ballHandler;
  private static ArmFeedforward pivotFF = new ArmFeedforward(Constants.pivotFF.kS, Constants.pivotFF.kS, Constants.pivotFF.kV);
  private Timer timeout = new Timer();
  // Adjust this when it's appropriate
  private double notThereYetTime = 3.0;
  private double tolerance = 1.5;
  /** Creates a new SetAnglePID. 
   * 
   * 
  */
  public SetAnglePID(double angle, BallHandler bh) {
    super(
        // The controller that the command will use
        new PIDController(Constants.pivotPIDConsts.pidP, Constants.pivotPIDConsts.pidI, Constants.pivotPIDConsts.pidD),
        // This should return the measurement
        bh::getPivotPosition,
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output - sign is specific for the hood
        output -> bh.setPivotPower(output + (pivotFF.calculate(angle*(Math.PI/180.0), 0.0)/12.0)), 
        bh);
    this.ballHandler = bh;
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
    ballHandler.setPivotPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return (getController().atSetpoint() | timeout.get() >= notThereYetTime);
  }
}
