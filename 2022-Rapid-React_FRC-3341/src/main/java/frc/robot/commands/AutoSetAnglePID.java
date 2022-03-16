// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSetAnglePID extends CommandBase {
  private double angle;

  /** Creates a new SetAnglePID. 
   * @param angle - The angle in degrees, measured from the horizontal position
  */
  
  public AutoSetAnglePID(double angle) {
    this.angle = angle;
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.getBallHandler().setPivotAngle(angle);
  }

  // Returns true when the command should end.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return RobotContainer.getBallHandler().atSetpoint();
  }
}