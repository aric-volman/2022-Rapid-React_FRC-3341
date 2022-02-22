// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import frc.robot.RobotContainer;



import frc.robot.Constants;

public class BallHandler extends SubsystemBase {

  private static BallHandler ballHandler;

  //fill in later
  private double wheelDiameter = 0.1;
  private double wheelCircumference = wheelDiameter*Math.PI;

  private double ticksToMeters = wheelCircumference / 4096;
  //private double ticksToDegrees = 4096*360;

  private double flywheelTolerance;

  //change ports when ready to start testing
  private final WPI_TalonSRX leftflywheel = new WPI_TalonSRX(Constants.MotorPorts.port1);
  private final WPI_TalonSRX rightflywheel = new WPI_TalonSRX(Constants.MotorPorts.port2);
  private final WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.MotorPorts.port3);
  private final WPI_TalonSRX roller = new WPI_TalonSRX(Constants.MotorPorts.port5);

  private ShuffleboardTab tab = Shuffleboard.getTab("Flywheel PID");
  private NetworkTableEntry leftflywheelTestInputPIDP = tab.add("Left Flywheel PID P", Constants.flywheelPIDConsts.pidP).getEntry();
  private NetworkTableEntry leftflywheelTestInputPIDI = tab.add("Left Flywheel PID I", Constants.flywheelPIDConsts.pidI).getEntry();
  private NetworkTableEntry leftflywheelTestInputPIDD = tab.add("Left Flywheel PID D", Constants.flywheelPIDConsts.pidD).getEntry();
  // Override with testing Constants for flywheel
  private final PIDController leftflywheelPID = new PIDController(leftflywheelTestInputPIDP.getDouble(Constants.flywheelPIDConsts.pidP), leftflywheelTestInputPIDI.getDouble(Constants.flywheelPIDConsts.pidI), leftflywheelTestInputPIDD.getDouble(Constants.flywheelPIDConsts.pidD));
  private SimpleMotorFeedforward leftflywheelFF = new SimpleMotorFeedforward(Constants.LeftflywheelFF.kS, Constants.LeftflywheelFF.kV, Constants.LeftflywheelFF.kA);
 
  public BallHandler() {
    pivot.configFactoryDefault();
    pivot.setInverted(false);
    pivot.setNeutralMode(NeutralMode.Brake);
    pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    pivot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.setNeutralMode(NeutralMode.Brake);
    pivot.configNeutralDeadband(0.007);
    roller.configFactoryDefault();
    roller.setInverted(false);
    leftflywheel.configFactoryDefault();
    leftflywheel.setInverted(false);
    leftflywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    rightflywheel.configFactoryDefault();
    rightflywheel.setInverted(true);
    rightflywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    leftflywheelPID.setTolerance(flywheelTolerance);

  }

  public BallHandler getInstance() {
    if(ballHandler == null) {
      ballHandler = new BallHandler();
    }
    return ballHandler;
  }

  

  public void resetFlywheelEncoders(){
    leftflywheel.setSelectedSensorPosition(0, 0, 10);
    rightflywheel.setSelectedSensorPosition(0, 0, 10);
    
  }

  public void resetPivotEncoders() {
    pivot.setSelectedSensorPosition(0, 0, 10);
  }

  public double getFlywheelPosition(){
    return (((leftflywheel.getSelectedSensorPosition(0) + rightflywheel.getSelectedSensorPosition(0))/2) * (ticksToMeters));
  }

  public double getPivotPosition(){
    return (((-1.0*pivot.getSelectedSensorPosition(0)/4096.0)*360.0*(16.0/50.0))); // 16.0 to 50.0 gear ratio after encoder
    //return pivot.getSelectedSensorPosition(0);
  }

  public boolean isForwardLimitClosed() {
    if(pivot.isFwdLimitSwitchClosed() == 0) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean isReverseLimitClosed() {
    if(pivot.isRevLimitSwitchClosed() == 0) {
      return true;
    }
    else {
      return false;
    }
  }

  public double getVelocity(){
    //return (((leftflywheel.getSensorCollection().getPulseWidthVelocity() + rightflywheel.getSensorCollection().getPulseWidthVelocity())/2) * (ticksToMeters));
    return ((leftflywheel.getSelectedSensorVelocity() * 10)/4096.0)*wheelCircumference;
 }

 public double getTicks(WPI_TalonSRX motor) {
   return motor.getSelectedSensorPosition();
 }

 public double getRollerTicks() {
  return roller.getSelectedSensorPosition();
}

public double getFlywheelCurrent() {
  return leftflywheel.getStatorCurrent();
}

  public void setFlywheelPower(double speed) {
    leftflywheel.set(speed);
    //rightflywheel.set(speed);
  }

  public void setFlywheelConstantVelocity(double velocity) {
    //Need to look at possible unit conversions (probably sticking to 100ms intervals). Is *ticksToMeters the right way?
    //double power = leftflywheelPID.calculate(leftflywheel.getSelectedSensorVelocity() * ticksToMeters, velocity);
    leftflywheel.set((leftflywheelFF.calculate(velocity))/12.0 + leftflywheelPID.calculate(getVelocity(), velocity)); //DIVIDE BY THE VOLTAGE!!!
    //rightflywheel.set(power);
  }

  public boolean flywheelWithinErrorMargin() {
    return (leftflywheelPID.atSetpoint());
  }

  public void setPivotPower(double speed) {
    pivot.set(speed);
  }

  public void setRollerPower(double speed) {
    roller.set(speed);
  }

  public double getRollerPower() {
    return roller.get();
  }

  public double getPivotPower() {
    return pivot.get();
  }

  public double getFlywheelPower() {
    return leftflywheel.get();
  }

  public void setPivotBrake() {
    pivot.setNeutralMode(NeutralMode.Brake);
  }

  public void setPivotCoast() {
    pivot.setNeutralMode(NeutralMode.Coast);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Ticks: ", getTicks(leftflywheel));
    SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
    SmartDashboard.putNumber("Flywheel Power", getFlywheelPower());
    SmartDashboard.putNumber("Roller Ticks: ", getRollerTicks());
    SmartDashboard.putNumber("Roller Power: ", getRollerPower());

    SmartDashboard.putNumber("Pivot Angle: ", getPivotPosition());
    SmartDashboard.putNumber("Pivot Power", getPivotPower());
    //A return of 'false' means that the limit switch is active
    SmartDashboard.putBoolean("Forward Limit Switch: ", isForwardLimitClosed());
    SmartDashboard.putBoolean("Reverse Limit Switch: ", isReverseLimitClosed());
    if (pivot.isFwdLimitSwitchClosed() == 0) {
      pivot.setSelectedSensorPosition(0, 0, 10);
    }
    setPivotPower(RobotContainer.getJoystick().getY());
    //setFlywheelPower(RobotContainer.getJoystick().getY());
    if (RobotContainer.getJoystick().getRawButton(6)) {
      SmartDashboard.putBoolean("Button 6", RobotContainer.getJoystick().getRawButton(6));
      pivot.setNeutralMode(NeutralMode.Coast);
    }
    leftflywheelPID.setPID(leftflywheelTestInputPIDP.getDouble(Constants.flywheelPIDConsts.pidP), leftflywheelTestInputPIDI.getDouble(Constants.flywheelPIDConsts.pidI), leftflywheelTestInputPIDD.getDouble(Constants.flywheelPIDConsts.pidD));
    
  }


}
