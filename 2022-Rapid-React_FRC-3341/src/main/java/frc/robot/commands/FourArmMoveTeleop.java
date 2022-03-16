// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm1;

public class FourArmMoveTeleop extends CommandBase {
  /** Creates a new FourArmMoveTeleop. */
  private Arm1 arm1, arm2, arm3, arm4;

  public FourArmMoveTeleop(Arm1 arm1, Arm1 arm2, Arm1 arm3, Arm1 arm4) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm1 = arm1;
    this.arm2 = arm2;
    this.arm3 = arm3;
    this.arm4 = arm4;

    addRequirements(arm1, arm2, arm3, arm4);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm1.setExtBrake(false);
    arm2.setExtBrake(false);
    arm3.setExtBrake(false);
    arm4.setExtBrake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double POV = RobotContainer.getJoy3().getPOV();
    
    if((POV >= 0 && POV < 70) || (POV >= 290)){
      /*
      if(RobotContainer.getJoy1().getRawButtonPressed(1)){
        //power level will be changed with testing
        arm1.extend(0.1);
        arm2.extend(0.1);
        arm3.extend(0.1);
        arm4.extend(0.1);
      }
      */
      arm1.extend(1);
      arm2.extend(1);
      arm3.extend(1);
      arm4.extend(1);
      arm1.setExtBrake(false);
      arm2.setExtBrake(false);
      arm3.setExtBrake(false);
      arm4.setExtBrake(false);
    }
    else if(POV >= 110 && POV <= 250){
      /*
      if(RobotContainer.getJoy1().getRawButtonPressed(1)){
        //power level will be changed with testing
        arm1.extend(-0.1);
        arm2.extend(-0.1);
        arm3.extend(-0.1);
        arm4.extend(-0.1);
      }
      */
      arm1.extend(-1);
      arm2.extend(-1);
      arm3.extend(-1);
      arm4.extend(-1);
      arm1.setExtBrake(false);
      arm2.setExtBrake(false);
      arm3.setExtBrake(false);
      arm4.setExtBrake(false);
    } else {
      arm1.extend(0);
      arm2.extend(0);
      arm3.extend(0);
      arm4.extend(0);
      arm1.setExtBrake(true);
      arm2.setExtBrake(true);
      arm3.setExtBrake(true);
      arm4.setExtBrake(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm1.extend(0);
    arm2.extend(0);
    arm3.extend(0);
    arm4.extend(0);
    arm1.setExtBrake(true);
    arm2.setExtBrake(true);
    arm3.setExtBrake(true);
    arm4.setExtBrake(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
