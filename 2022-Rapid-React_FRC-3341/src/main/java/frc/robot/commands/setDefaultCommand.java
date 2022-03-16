// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm1;
import frc.robot.subsystems.DriveTrain;

public class setDefaultCommand extends CommandBase {
  /** Creates a new setDefaultCommand. */
  private Arm1 arm1, arm2, arm3, arm4;

  public setDefaultCommand(Arm1 arm1, Arm1 arm2, Arm1 arm3, Arm1 arm4, DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm1 = arm1;
    this.arm2 = arm2;
    this.arm3 = arm3;
    this.arm4 = arm4;
    addRequirements(arm1, arm2, arm3, arm4, dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmMoveTeleop frontLeftCom = new ArmMoveTeleop(arm1, RobotContainer.getJoy1());
    ArmMoveTeleop frontRightCom = new ArmMoveTeleop(arm2, RobotContainer.getJoy2());
    ArmMoveTeleop backLeftCom = new ArmMoveTeleop(arm3, RobotContainer.getJoy3());
    ArmMoveTeleop backRightCom = new ArmMoveTeleop(arm4, RobotContainer.getJoy4());
    arm1.setDefaultCommand(frontLeftCom);
    arm2.setDefaultCommand(frontRightCom);
    arm3.setDefaultCommand(backLeftCom);
    arm4.setDefaultCommand(backRightCom);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
