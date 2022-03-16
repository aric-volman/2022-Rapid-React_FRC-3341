// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.RobotContainer;

public class AutoSimpleShoot extends CommandBase {

  private double rollerPower = 1.0;
  private double cargoIsLaunchedTime = 3.0; // Arguably the most important timer
  private boolean isFlywheelAtSpeed;

  Timer cargoTimer = new Timer();

  /** Creates a new EncoderShoot. 
   *
  */
  public AutoSimpleShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    cargoTimer.start();
    cargoTimer.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFlywheelAtSpeed = false;
    RobotContainer.getBallHandler().resetFlywheelEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Comment these out if you feel PID is needed
    // ballHandler.setFlywheelConstantVelocity(velocity);
    // if (ballHandler.flywheelWithinErrorMargin()) {
    if (RobotContainer.getBallHandler().getAverageRPM() < 2200) {
      RobotContainer.getBallHandler().setRollerPower(0.0);
      RobotContainer.getBallHandler().setFlywheelPower(1.0);
    } else if (RobotContainer.getBallHandler().getAverageRPM() >= 2200 && !isFlywheelAtSpeed) { // RPM based
      RobotContainer.getBallHandler().setRollerPower(rollerPower);
      isFlywheelAtSpeed = true;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getBallHandler().setFlywheelPower(0.0);
    RobotContainer.getBallHandler().setRollerPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (cargoTimer.get() >= cargoIsLaunchedTime);
  }
}