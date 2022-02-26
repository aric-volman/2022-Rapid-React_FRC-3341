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

import frc.robot.Constants;

public class BallHandler extends SubsystemBase {

  private static BallHandler ballHandler;

  private double wheelDiameter = 0.1; // Diameter in meters
  private double wheelCircumference = wheelDiameter*Math.PI;

  private double flywheelTolerance = 0.05; // Tolerance in m/s

  // Change ports later...
  private final WPI_TalonSRX leftFlywheel = new WPI_TalonSRX(Constants.MotorPorts.leftFlywheelPort);
  private final WPI_TalonSRX rightFlywheel = new WPI_TalonSRX(Constants.MotorPorts.rightFlywheelPort);
  
  private final WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.MotorPorts.pivotPort);
  private final WPI_VictorSPX roller = new WPI_VictorSPX(Constants.MotorPorts.rollerPort);

  private ShuffleboardTab tab = Shuffleboard.getTab("Flywheel PID");
  
  private NetworkTableEntry leftFlywheelTestInputPIDP = tab.add("Left Flywheel PID P", Constants.leftFlywheelPIDConsts.pidP).getEntry();
  private NetworkTableEntry leftFlywheelTestInputPIDI = tab.add("Left Flywheel PID I", Constants.leftFlywheelPIDConsts.pidI).getEntry();
  private NetworkTableEntry leftFlywheelTestInputPIDD = tab.add("Left Flywheel PID D", Constants.leftFlywheelPIDConsts.pidD).getEntry();

  private NetworkTableEntry rightFlywheelTestInputPIDP = tab.add("Right Flywheel PID P", Constants.rightFlywheelPIDConsts.pidP).getEntry();
  private NetworkTableEntry rightFlywheelTestInputPIDI = tab.add("Right Flywheel PID I", Constants.rightFlywheelPIDConsts.pidI).getEntry();
  private NetworkTableEntry rightFlywheelTestInputPIDD = tab.add("Right Flywheel PID D", Constants.rightFlywheelPIDConsts.pidD).getEntry();

  // Overriden with testing Constants for flywheel
  private final PIDController leftFlywheelPID = new PIDController(leftFlywheelTestInputPIDP.getDouble(Constants.leftFlywheelPIDConsts.pidP), leftFlywheelTestInputPIDI.getDouble(Constants.leftFlywheelPIDConsts.pidI), leftFlywheelTestInputPIDD.getDouble(Constants.leftFlywheelPIDConsts.pidD));
  private final PIDController rightFlywheelPID = new PIDController(rightFlywheelTestInputPIDP.getDouble(Constants.rightFlywheelPIDConsts.pidP), rightFlywheelTestInputPIDI.getDouble(Constants.rightFlywheelPIDConsts.pidI), rightFlywheelTestInputPIDD.getDouble(Constants.rightFlywheelPIDConsts.pidD));
 
  private SimpleMotorFeedforward leftFlywheelFF = new SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);
  private SimpleMotorFeedforward rightFlywheelFF = new SimpleMotorFeedforward(Constants.rightFlywheelFF.kS, Constants.rightFlywheelFF.kV, Constants.rightFlywheelFF.kA);
 
  public BallHandler() {
    pivot.configFactoryDefault();
    pivot.setInverted(false);
    pivot.setNeutralMode(NeutralMode.Brake);
    pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    pivot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.setNeutralMode(NeutralMode.Brake);
    pivot.configNeutralDeadband(0.007); // Configure this later based upon lowest PID output value

    roller.configFactoryDefault();
    roller.setInverted(false);

    leftFlywheel.configFactoryDefault();
    leftFlywheel.setInverted(false);
    leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    rightFlywheel.configFactoryDefault();
    rightFlywheel.setInverted(true);
    rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    // leftFlywheel.setInverted(true);
    // rightFlywheel.setInverted(true);

    leftFlywheelPID.setTolerance(flywheelTolerance);
    rightFlywheelPID.setTolerance(flywheelTolerance);

  }

  public BallHandler getInstance() {
    if(ballHandler == null) {
      ballHandler = new BallHandler();
    }
    return ballHandler;
  }

  public double getTicks(WPI_TalonSRX motor) {
    return motor.getSelectedSensorPosition();
  }

  // FLYWHEEL METHODS --------------------------------------------------------------------

  public void resetFlywheelEncoders(){
    leftFlywheel.setSelectedSensorPosition(0, 0, 10);
    rightFlywheel.setSelectedSensorPosition(0, 0, 10);
  }

  public double getLeftVelocity(){
    return ((leftFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*wheelCircumference;
  }

  public double getRightVelocity(){
  return ((rightFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*wheelCircumference;
  }

  public double getLeftFlywheelPower() {
    return leftFlywheel.get();
  }

  public double getRightFlywheelPower() {
    return rightFlywheel.get();
  }
  
  public void setFlywheelPower(double speed) {
    leftFlywheel.set(speed);
    rightFlywheel.set(speed);
  }

  public void setFlywheelConstantVelocity(double velocity) {
    leftFlywheel.set((leftFlywheelFF.calculate(velocity))/12.0 + leftFlywheelPID.calculate(getLeftVelocity(), velocity)); //DIVIDE BY THE VOLTAGE!!!
    rightFlywheel.set((rightFlywheelFF.calculate(velocity))/12.0 + rightFlywheelPID.calculate(getRightVelocity(), velocity)); //DIVIDE BY THE VOLTAGE!!!
  }

  public boolean flywheelWithinErrorMargin() {
    return (leftFlywheelPID.atSetpoint() && rightFlywheelPID.atSetpoint());
  }

  public double getFlywheelCurrent() {
    return (leftFlywheel.getStatorCurrent() + rightFlywheel.getStatorCurrent())/2.0;
  }

  // PIVOT METHODS --------------------------------------------------------------------------

  public void resetPivotEncoders() {
    pivot.setSelectedSensorPosition(0, 0, 10);
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

  public double getPivotPosition(){
    // return (((-1.0*pivot.getSelectedSensorPosition(0)/4096.0)*360.0*(16.0/50.0))); // 16.0 to 50.0 gear ratio after encoder
    return (((-1.0*pivot.getSelectedSensorPosition(0)/4096.0)*360.0)); // Ok we actually don't know this one, plus the negative might not be needed
    // return pivot.getSelectedSensorPosition(0);
  }

  public void setPivotPower(double power) {
    pivot.set(power);
  }

  public double getPivotPower() {
    return pivot.get();
  }

  public void setPivotBrake() {
    pivot.setNeutralMode(NeutralMode.Brake);
  }

  public void setPivotCoast() {
    pivot.setNeutralMode(NeutralMode.Coast);
  }

  // ROLLER METHODS -------------------------------------------------------------------------

  public void setRollerPower(double power) {
    roller.set(power);
  }

  public double getRollerPower() {
    return roller.get();
  }

  public double getRollerTicks() {
    return roller.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Left Flywheel Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Left Flywheel Power", getLeftFlywheelPower());
    SmartDashboard.putNumber("Left Flywheel Ticks: ", getTicks(leftFlywheel));

    SmartDashboard.putNumber("Right Flywheel Velocity", getRightVelocity());
    SmartDashboard.putNumber("Right Flywheel Power", getRightFlywheelPower());
    SmartDashboard.putNumber("Right Flywheel Ticks: ", getTicks(rightFlywheel));
    
    SmartDashboard.putNumber("Roller Ticks: ", getRollerTicks());
    SmartDashboard.putNumber("Roller Power: ", getRollerPower());

    SmartDashboard.putNumber("Pivot Angle: ", getPivotPosition());
    SmartDashboard.putNumber("Pivot Power", getPivotPower());

    // A return of 'false' means that the limit switch is active
    SmartDashboard.putBoolean("Forward Limit Switch: ", isForwardLimitClosed());
    SmartDashboard.putBoolean("Reverse Limit Switch: ", isReverseLimitClosed());
    
    if (pivot.isFwdLimitSwitchClosed() == 0) {
      pivot.setSelectedSensorPosition(0, 0, 10);
    }

    //setPivotPower(RobotContainer.getJoystick().getY());
    //setFlywheelPower(RobotContainer.getJoystick().getY());
    //setRollerPower(RobotContainer.getJoystick().getY());
    
    leftFlywheelPID.setPID(leftFlywheelTestInputPIDP.getDouble(Constants.leftFlywheelPIDConsts.pidP), leftFlywheelTestInputPIDI.getDouble(Constants.leftFlywheelPIDConsts.pidI), leftFlywheelTestInputPIDD.getDouble(Constants.leftFlywheelPIDConsts.pidD));
    rightFlywheelPID.setPID(rightFlywheelTestInputPIDP.getDouble(Constants.rightFlywheelPIDConsts.pidP), rightFlywheelTestInputPIDI.getDouble(Constants.rightFlywheelPIDConsts.pidI), rightFlywheelTestInputPIDD.getDouble(Constants.rightFlywheelPIDConsts.pidD));
  
  }


}
