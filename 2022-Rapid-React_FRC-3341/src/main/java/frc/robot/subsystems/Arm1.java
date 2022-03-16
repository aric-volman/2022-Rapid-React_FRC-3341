// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;

public class Arm1 extends SubsystemBase {
  /** Creates a new Arm1. */

  private final WPI_TalonSRX extend, rotate;
  private final DigitalInput input;

  private int armExtPos;
  private double armPower;
  private boolean armExtPrevState;
  private final int minArmState = 0;
  private final int maxArmState = 5;
  private String armName;

  public Arm1(int extendID, int rotateID, int inputID, String armName) {

      armExtPos = 0;
      armPower = 0;
      this.armName = armName;


      extend = new WPI_TalonSRX(extendID);
      rotate = new WPI_TalonSRX(rotateID);
      input = new DigitalInput(inputID);
      armExtPrevState = isOnTape();
      
      extend.configFactoryDefault();
      rotate.configFactoryDefault();

      rotate.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
      rotate.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
      rotate.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
      extend.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

      if (armName.equals("FrontLeft") || armName.equals("BackLeft")) {
        rotate.setInverted(true);
      } else {
        rotate.setInverted(false);
      }

      setExtBrake(true);
      setRotBrake(true);
  }

  public String getArmName(){
    return armName;
  }

  //EXTENSION METHODS
  public void extend(double power){
    extend.set(ControlMode.PercentOutput, power);
    //armPower = power;
  }

  public double getExtCurrent(){
    return extend.getSupplyCurrent();
  }

  public void setExtBrake(boolean isBrake){
    if(isBrake) extend.setNeutralMode(NeutralMode.Brake);
    else extend.setNeutralMode(NeutralMode.Coast);
  }

  public int isExtLSClosed(){
    return extend.isRevLimitSwitchClosed();
  }


  //ROTATION METHODS
  public void rotate(double power){
    rotate.set(ControlMode.PercentOutput, power);
  }

  public int isRotFwdLSClosed(){
    return rotate.isFwdLimitSwitchClosed();
  }

  public int isRotRevLSClosed(){
    return rotate.isRevLimitSwitchClosed();
  }

  public double getRotTicks(){
    return rotate.getSelectedSensorPosition();
  }

  public double getRotPosition(){
    return getRotTicks() * (360.0 / 4096.0);
  }

  public void resetArm(){
    rotate.setSelectedSensorPosition(0, 0, 10);
  }

  public void setRotBrake(boolean isBrake){
    if(isBrake) extend.setNeutralMode(NeutralMode.Brake);
    else extend.setNeutralMode(NeutralMode.Coast);
  }

  //COUNTING METHODS
  public boolean isOnTape(){
    return input.get();
  }

  public int getArmExtPos(){
    return armExtPos;
  }

  public int getArmMaxPos(){
    return maxArmState;
  }

  public int getArmMinPos(){
    return minArmState;
  }

  public void armCount(){
    boolean isOnTape = isOnTape();

    if(!armExtPrevState && isOnTape){
      if(armPower > 0) armExtPos++;
      else if(armPower < 0) armExtPos--;
    }
    armExtPrevState = isOnTape;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    armCount();

    //make sure it's fwd not rev to be designated as 0 (esp for PID)
    if(isRotFwdLSClosed() == 0){
      resetArm();
    }

    //extend(RobotContainer.getJoy2().getY());

    SmartDashboard.putNumber(armName + "RotLSFwd", isRotFwdLSClosed());
    SmartDashboard.putNumber(armName + "RotLSRev", isRotRevLSClosed());
    SmartDashboard.putNumber(armName + "ExtLS", isExtLSClosed());
    SmartDashboard.putNumber(armName + "Current", getExtCurrent());
    SmartDashboard.putNumber(armName + "TapeCount", getArmExtPos());
    SmartDashboard.putBoolean(armName + "ReflectiveSensor", isOnTape());

    SmartDashboard.putNumber("joyY", RobotContainer.getJoy1().getY());
    SmartDashboard.putNumber("POV", RobotContainer.getJoy1().getPOV());

    SmartDashboard.putBoolean("but5", RobotContainer.getJoy1().getRawButton(5));
    SmartDashboard.putBoolean("but6", RobotContainer.getJoy1().getRawButton(6));
    SmartDashboard.putBoolean("but3", RobotContainer.getJoy1().getRawButton(3));
    SmartDashboard.putBoolean("but4", RobotContainer.getJoy1().getRawButton(4));
  }
}
