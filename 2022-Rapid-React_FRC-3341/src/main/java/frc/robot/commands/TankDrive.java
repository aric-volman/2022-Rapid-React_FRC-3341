// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

  

/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  private final DriveTrain _driveTrain;
  private final Joystick _leftJoystick;
  private final Joystick _rightJoystick;
  private TalonSRX ballHandler;

  public TankDrive(DriveTrain dt, Joystick leftJ, Joystick rightJ) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    _leftJoystick = leftJ;
    _rightJoystick = rightJ;
    /*
    ballHandler = new TalonSRX(Constants.BallPorts.BallPivot);
    ballHandler.configFactoryDefault();
    ballHandler.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    ballHandler.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);*/

    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    if (RobotContainer.getJoy1().getRawButtonPressed(12) || RobotContainer.getJoy2().getRawButtonPressed(12) || RobotContainer.getJoy3().getRawButtonPressed(12) || RobotContainer.getJoy4().getRawButtonPressed(12)) {
      RobotContainer.switchIsDriving();
    }

    if (RobotContainer.getIsDriving()) {
      _driveTrain.tankDrive(-0.8 * _leftJoystick.getRawAxis(Constants.JoystickAxis.YAxis), -0.8 * _rightJoystick.getRawAxis(Constants.JoystickAxis.YAxis));
      
    } else{
       if(ballHandler.isRevLimitSwitchClosed() == 1){
         //SmartDashboard.putString("BallHandlerPID:", "settingPowertoBH");
         //ballHandler.set(ControlMode.PercentOutput, -0.2);
       }
    }
  }
                        
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
