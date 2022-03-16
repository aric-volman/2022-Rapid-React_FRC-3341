// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm1;

public class ArmMoveTeleop extends CommandBase {
  /** Creates a new ArmMoveTeleop. */
  private Arm1 armSub;
  private Joystick joy;
  private boolean isHolding;

  public ArmMoveTeleop(Arm1 armSubsystem, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    armSub = armSubsystem;
    this.joy = joy;
    isHolding = false;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSub.setExtBrake(false);
    armSub.setRotBrake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      /*
        POV + trigger control extension/retraction (POV designates direction and trigger sets power)
        Y-axis controls rotation
      */
     // SmartDashboard.putString("armNum Command", armSub.getName());

      if (!RobotContainer.getIsDriving()) {
        double joyY = joy.getY();
      //double joyX = RobotContainer.getJoy1().getX();
      double POV = joy.getPOV();

      //joyY *= -1;
      
      if((POV >= 0 && POV < 70) || (POV >= 290)){
        /*
        if(RobotContainer.getJoy1().getRawButtonPressed(1)){
          //power level will be changed with testing
          armSub.extend(0.3);
        }else{
          armSub.extend(0);
        }
        */
        armSub.extend(1);
        armSub.setExtBrake(false);
        
      } else if(POV >= 110 && POV <= 250){
        /*
        if(RobotContainer.getJoy1().getRawButtonPressed(1)){
          //power level will be changed with testing
          armSub.extend(-0.3);
        }else{
          armSub.extend(0);
        }
        */
        armSub.extend(-1);
        armSub.setExtBrake(false);
      } else {
        armSub.extend(0);
        armSub.setExtBrake(true);
      }

      if(joy.getRawButtonPressed(1) ){
        isHolding = !isHolding;  
      }

      if(isHolding) armSub.extend(-0.2);

      if (Math.abs(joyY) > 0.1) {
        armSub.setRotBrake(false);
        armSub.rotate(joyY*0.2);
      } else {
        armSub.rotate(0);
        armSub.setRotBrake(true);
      }

      SmartDashboard.putNumber("ARM ROTATION POWER", joyY*0.2);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*armSub.extend(0);
    armSub.rotate(0);
    armSub.setExtBrake(true);
    armSub.setRotBrake(true);*/
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
