// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;

//1 = Front Left, 2 = Front Right, 3 = Back Left, 4 = Back Right test!

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  //private final WPI_TalonSRX extend;
  //private final WPI_TalonSRX rotate;

  private final WPI_TalonSRX frontLeftRot, frontRightRot, backLeftRot, backRightRot;
  private final WPI_TalonSRX frontLeftExt, frontRightExt, backLeftExt, backRightExt;
  private final DigitalInput input1, input2, input3, input4;

  //arm variables
  private int armExtPos1, armExtPos2, armExtPos3, armExtPos4;
  private double armPower1, armPower2, armPower3, armPower4;
  private boolean armExtPrevState1, armExtPrevState2, armExtPrevState3, armExtPrevState4;
  private boolean but5, but6, but3, but4;
  private final int minArmState = 0;
  private final int maxArmState = 5;


  public Arm() {
    
    //these IDs need to be changed
    input1 = new DigitalInput(0);
    input2 = new DigitalInput(1);
    input3 = new DigitalInput(2);
    input4 = new DigitalInput(3);

    armExtPos1 = 0;
    armExtPos2 = 0;
    armExtPos3 = 0;
    armExtPos4 = 0;

    armPower1 = 0;
    armPower2 = 0;
    armPower3 = 0;
    armPower4 = 0;
    
    armExtPrevState1 = isOnTape(1);
    armExtPrevState2 = isOnTape(2);
    armExtPrevState3 = isOnTape(3);
    armExtPrevState4 = isOnTape(4);


    //talon configurations
    frontLeftRot = new WPI_TalonSRX(Constants.ArmPorts.FrontLeftArmRot);
    frontRightRot = new WPI_TalonSRX(Constants.ArmPorts.FrontRightArmRot);
    backLeftRot = new WPI_TalonSRX(Constants.ArmPorts.BackLeftArmRot);
    backRightRot = new WPI_TalonSRX(Constants.ArmPorts.BackRightArmRot);

    frontLeftExt = new WPI_TalonSRX(Constants.ArmPorts.FrontLeftArmExt);
    frontRightExt = new WPI_TalonSRX(Constants.ArmPorts.FrontRightArmExt);
    backLeftExt = new WPI_TalonSRX(Constants.ArmPorts.BackLeftArmExt);
    backRightExt = new WPI_TalonSRX(Constants.ArmPorts.BackRightArmExt);

    frontLeftRot.configFactoryDefault();
    frontRightRot.configFactoryDefault();
    backLeftRot.configFactoryDefault();
    backRightRot.configFactoryDefault();
    frontLeftExt.configFactoryDefault();
    frontRightExt.configFactoryDefault();
    backLeftExt.configFactoryDefault();
    backRightExt.configFactoryDefault();
    
    frontLeftRot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    frontLeftRot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    frontRightRot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    frontRightRot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    backLeftRot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    backLeftRot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    backRightRot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    backRightRot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

    frontLeftExt.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    frontRightExt.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    backLeftExt.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    backRightExt.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    

    /*
    extend = new WPI_TalonSRX(1);
    rotate = new WPI_TalonSRX(4);

    extend.setInverted(false);
    rotate.setInverted(false);

    extend.configFactoryDefault();
    rotate.configFactoryDefault();

    rotate.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    rotate.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    rotate.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    */
  }

  //extension methods
  public void extend(int motorNum, double power) {
    if (motorNum == 1) {
      frontLeftExt.set(ControlMode.PercentOutput, power);
      armPower1 = power;
    } else if (motorNum == 2) {
      frontRightExt.set(ControlMode.PercentOutput, power);
      armPower2 = power;
    } else if (motorNum == 3) {
      backLeftExt.set(ControlMode.PercentOutput, power);
      armPower3 = power;
    } else if (motorNum == 4) {
      backRightExt.set(ControlMode.PercentOutput, power);
      armPower4 = power;
    }
  }

  public double getArmExtCurrent(int motorNum) {
    if (motorNum == 1) {
      return frontLeftExt.getSupplyCurrent();
    } else if (motorNum == 2) {
      return frontRightExt.getSupplyCurrent();
    } else if (motorNum == 3) {
      return backLeftExt.getSupplyCurrent();
    } else {
      return backRightExt.getSupplyCurrent();
    }
  }

  public void setExtBrake(int motorNum, boolean brake) {
    NeutralMode mode;
    if (brake) {
      mode = NeutralMode.Brake;
    } else {
      mode = NeutralMode.Coast;
    }

    if (motorNum == 1) {
      frontLeftExt.setNeutralMode(mode);
    } else if (motorNum == 2) {
      frontRightExt.setNeutralMode(mode);
    } else if (motorNum == 3) {
      backLeftExt.setNeutralMode(mode);
    } else if (motorNum == 4){
      backRightExt.setNeutralMode(mode);
    }
  }

  public int isExtLSClosed(int motorNum) {
    if (motorNum == 1) {
      return frontLeftExt.isRevLimitSwitchClosed();
    } else if (motorNum == 2) {
      return frontRightExt.isRevLimitSwitchClosed();
    } else if (motorNum == 3) {
      return backLeftExt.isRevLimitSwitchClosed();
    } else {
      return backRightExt.isRevLimitSwitchClosed();
    }
  }


  //rotation methods
  public void rotate(int motorNum, double power) {
    if (motorNum == 1) {
      frontLeftRot.set(ControlMode.PercentOutput, power);
    } else if (motorNum == 2) {
      frontRightRot.set(ControlMode.PercentOutput, power);
    } else if (motorNum == 3) {
      backLeftRot.set(ControlMode.PercentOutput, power);
    } else if (motorNum == 4) {
      backRightRot.set(ControlMode.PercentOutput, power);
    }
  }

  
  public int isRotFwdLSClosed(int motorNum) {
    if (motorNum == 1) {
      return frontLeftRot.isFwdLimitSwitchClosed();
    } else if (motorNum == 2) {
      return frontRightRot.isFwdLimitSwitchClosed();
    } else if (motorNum == 3) {
      return backLeftRot.isFwdLimitSwitchClosed();
    } else {
      return backRightRot.isFwdLimitSwitchClosed();
    }
  }

  public int isRotRevLSClosed(int motorNum) {
    if (motorNum == 1) {
      return frontLeftRot.isRevLimitSwitchClosed();
    } else if (motorNum == 2) {
      return frontRightRot.isRevLimitSwitchClosed();
    } else if (motorNum == 3) {
      return backLeftRot.isRevLimitSwitchClosed();
    } else {
      return backRightRot.isRevLimitSwitchClosed();
    }
  }

  public double getArmTicks(int motorNum){
    if (motorNum == 1) {
      return frontLeftRot.getSelectedSensorPosition();
    } else if (motorNum == 2) {
      return frontRightRot.getSelectedSensorPosition();
    } else if (motorNum == 3) {
      return backLeftRot.getSelectedSensorPosition();
    } else {
      return backRightRot.getSelectedSensorPosition();
    }
  }

  public double getArmPosition(int motorNum){
    double convValue = 360 / 4096.0;
    if (motorNum == 1) {
      return getArmTicks(1) * convValue;
    } else if (motorNum == 2) {
      return getArmTicks(2) * convValue;
    } else if (motorNum == 3) {
      return getArmTicks(3) * convValue;
    } else {
      return getArmTicks(4) * convValue;
    }
  }

  public void resetArm(int motorNum){
    if (motorNum == 1) {
      frontLeftRot.setSelectedSensorPosition(0, 0, 10);
    } else if (motorNum == 2) {
      frontRightRot.setSelectedSensorPosition(0, 0, 10);
    } else if (motorNum == 3) {
      backLeftRot.setSelectedSensorPosition(0, 0, 10);
    } else if (motorNum == 4){
      backRightRot.setSelectedSensorPosition(0, 0, 10);
    }
  }
  
  
  //arm counting methods
  public boolean isOnTape(int motorNum){
    if (motorNum == 1) {
      return input1.get();
    } else if (motorNum == 2) {
      return input2.get();
    } else if (motorNum == 3) {
      return input3.get();
    } else {
      return input4.get();
    }
  }

  public int getArmExtPos(int motorNum){
    if (motorNum == 1) {
      return armExtPos1;
    } else if (motorNum == 2) {
      return armExtPos2;
    } else if (motorNum == 3) {
      return armExtPos3;
    } else {
      return armExtPos4;
    }
  }

  public int getArmMaxPos(){
    return maxArmState;
  }

  public int getArmMinPos(){
    return minArmState;
  }

  public void armCount(){
    boolean isOnTape1 = isOnTape(1);
    boolean isOnTape2 = isOnTape(2);
    boolean isOnTape3 = isOnTape(3);
    boolean isOnTape4 = isOnTape(4);

    if(!armExtPrevState1 && isOnTape1){
      if(armPower1 > 0) armExtPos1++;
      else if(armPower1 < 0) armExtPos1--;
    }
    armExtPrevState1 = isOnTape1;


    if(!armExtPrevState2 && isOnTape2){
      if(armPower2 > 0) armExtPos2++;
      else if(armPower2 < 0) armExtPos2--;
    }
    armExtPrevState2 = isOnTape2;


    if(!armExtPrevState3 && isOnTape3){
      if(armPower3 > 0) armExtPos3++;
      else if(armPower3 < 0) armExtPos3--;
    }
    armExtPrevState3 = isOnTape3;


    if(!armExtPrevState4 && isOnTape4){
      if(armPower4 > 0) armExtPos4++;
      else if(armPower4 < 0) armExtPos4--;
    }
    armExtPrevState4 = isOnTape4;
  }
  

  @Override
  public void periodic() {

    armCount();

    
    

    //SmartDashboard.putNumber("PivotLSFwd", pivot.isFwdLimitSwitchClosed());
    //SmartDashboard.putNumber("PivotLSRev", pivot.isRevLimitSwitchClosed());

    /*SmartDashboard.putNumber("FrontLeftRotLSFwd", isRotFwdLSClosed(1));
    SmartDashboard.putNumber("FrontRightRotLSFwd", isRotFwdLSClosed(2));
    SmartDashboard.putNumber("BackLeftRotLSFwd", isRotFwdLSClosed(3));
    SmartDashboard.putNumber("BackRightRotLSFwd", isRotFwdLSClosed(4));

    SmartDashboard.putNumber("FrontLeftRotLSFRev", isRotRevLSClosed(1));
    SmartDashboard.putNumber("FrontRightRotLSRev", isRotRevLSClosed(2));
    SmartDashboard.putNumber("BackLeftRotLSRev", isRotRevLSClosed(3));
    SmartDashboard.putNumber("BackRightRotLSRev", isRotRevLSClosed(4));

    SmartDashboard.putNumber("FrontLeftExtLS", isExtLSClosed(1));
    SmartDashboard.putNumber("FrontRightExtLS", isExtLSClosed(2));
    SmartDashboard.putNumber("BackLeftExtLS", isExtLSClosed(3));
    SmartDashboard.putNumber("BackRightExtLS", isExtLSClosed(4));*/

    /*
    SmartDashboard.putNumber("FrontLeftCurr", getArmExtCurrent(1));
    SmartDashboard.putNumber("FrontRightCurr", getArmExtCurrent(2));
    SmartDashboard.putNumber("BackLeftCurr", getArmExtCurrent(3));
    SmartDashboard.putNumber("BackRightCurr", getArmExtCurrent(4));
    */

    //SmartDashboard.putNumber("ArmPower1", armPower1);

    /*SmartDashboard.putNumber("FrontLeftCount", getArmExtPos(1));
    SmartDashboard.putNumber("FrontRightCount", getArmExtPos(2));
    SmartDashboard.putNumber("BackLeftCount", getArmExtPos(3));
    SmartDashboard.putNumber("BackRightCount", getArmExtPos(4));*/
  }
}
