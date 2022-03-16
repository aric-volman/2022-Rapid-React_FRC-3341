// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class BallHandler extends SubsystemBase {

  private double wheelDiameter = 0.1; // Diameter in meters
  private double wheelCircumference = wheelDiameter*Math.PI;

  private double flywheelTolerance = 0.05; // Tolerance in m/s

  private double pivotTolerance = 1.0;

  private double encoderConst = 1.0; // Flips the sign of the angle if needed

  private double angle = 90.0; // In degrees

  private double angularVelocity = 45.0; // In degrees/s

  private double angularAcceleration = 15.0; // In degrees/s^2

  // In degrees, this is added, not subtracted
  // We set the reverse limit switch to zero and add 90 from there
  private double offset = Constants.angularOffset;

  private double maxHorizontalPower = -0.2; // Percent of volts, negative!?

  private Timer angleTimer = new Timer();

  private double angleTime = 0.2;

  private boolean override = true;

  // Change ports later...
  private final WPI_TalonSRX leftFlywheel = new WPI_TalonSRX(Constants.BallHandlerPorts.leftFlywheelPort);
  private final WPI_TalonSRX rightFlywheel = new WPI_TalonSRX(Constants.BallHandlerPorts.rightFlywheelPort);
  
  private final WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.BallHandlerPorts.pivotPort);
  private final WPI_VictorSPX roller = new WPI_VictorSPX(Constants.BallHandlerPorts.rollerPort);

  private ShuffleboardTab pidTab = Shuffleboard.getTab("Flywheel PID");

  // private ShuffleboardTab testTab = Shuffleboard.getTab("Testing Tab (Voltage)");
  
  private NetworkTableEntry leftFlywheelTestInputPIDP = pidTab.add("Left Flywheel PID P", Constants.leftFlywheelPIDConsts.pidP).getEntry();
  private NetworkTableEntry leftFlywheelTestInputPIDI = pidTab.add("Left Flywheel PID I", Constants.leftFlywheelPIDConsts.pidI).getEntry();
  private NetworkTableEntry leftFlywheelTestInputPIDD = pidTab.add("Left Flywheel PID D", Constants.leftFlywheelPIDConsts.pidD).getEntry();

  private NetworkTableEntry rightFlywheelTestInputPIDP = pidTab.add("Right Flywheel PID P", Constants.rightFlywheelPIDConsts.pidP).getEntry();
  private NetworkTableEntry rightFlywheelTestInputPIDI = pidTab.add("Right Flywheel PID I", Constants.rightFlywheelPIDConsts.pidI).getEntry();
  private NetworkTableEntry rightFlywheelTestInputPIDD = pidTab.add("Right Flywheel PID D", Constants.rightFlywheelPIDConsts.pidD).getEntry();

  // private NetworkTableEntry testMaxPower = testTab.add("Test Voltage", 0.5).getEntry();

  private NetworkTableEntry pivotTestInputPIDP = pidTab.add("Pivot PID P", Constants.pivotPIDConsts.pidP).getEntry();
  private NetworkTableEntry pivotTestInputPIDI = pidTab.add("Pivot PID I", Constants.pivotPIDConsts.pidI).getEntry();
  private NetworkTableEntry pivotTestInputPIDD = pidTab.add("Pivot PID D", Constants.pivotPIDConsts.pidD).getEntry();

  // Overriden with testing Constants for flywheels and pivot
  private final PIDController leftFlywheelPID = new PIDController(leftFlywheelTestInputPIDP.getDouble(Constants.leftFlywheelPIDConsts.pidP), leftFlywheelTestInputPIDI.getDouble(Constants.leftFlywheelPIDConsts.pidI), leftFlywheelTestInputPIDD.getDouble(Constants.leftFlywheelPIDConsts.pidD));
  private final PIDController rightFlywheelPID = new PIDController(rightFlywheelTestInputPIDP.getDouble(Constants.rightFlywheelPIDConsts.pidP), rightFlywheelTestInputPIDI.getDouble(Constants.rightFlywheelPIDConsts.pidI), rightFlywheelTestInputPIDD.getDouble(Constants.rightFlywheelPIDConsts.pidD));
  private final PIDController pivotPID = new PIDController(rightFlywheelTestInputPIDP.getDouble(Constants.pivotPIDConsts.pidP), rightFlywheelTestInputPIDI.getDouble(Constants.pivotPIDConsts.pidI), rightFlywheelTestInputPIDD.getDouble(Constants.pivotPIDConsts.pidD));

  private SimpleMotorFeedforward leftFlywheelFF = new SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);
  private SimpleMotorFeedforward rightFlywheelFF = new SimpleMotorFeedforward(Constants.rightFlywheelFF.kS, Constants.rightFlywheelFF.kV, Constants.rightFlywheelFF.kA);
  // private ArmFeedforward pivotFF = new ArmFeedforward(Constants.pivotFF.kS, Constants.pivotFF.kC, Constants.pivotFF.kV);
 
  public BallHandler() {
    pivot.configFactoryDefault();
    pivot.setInverted(false);
    pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    pivot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.configPeakCurrentLimit(20, 5);
    pivot.setNeutralMode(NeutralMode.Brake);

    // Motion Magic Velocity
    pivot.configMotionCruiseVelocity(((angularVelocity/360.0)*4096.0*10.0), 5);

    // Motion Magic Acceleration
    pivot.configMotionAcceleration(((angularAcceleration/360.0)*4096.0*10.0), 5);


    roller.configFactoryDefault();
    roller.setInverted(false);

    leftFlywheel.configFactoryDefault();
    leftFlywheel.setInverted(true);
    leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    rightFlywheel.configFactoryDefault();
    rightFlywheel.setInverted(false);
    rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    leftFlywheel.setInverted(true);
    // rightFlywheel.setInverted(true);

    leftFlywheelPID.setTolerance(flywheelTolerance);
    rightFlywheelPID.setTolerance(flywheelTolerance);
    pivotPID.setTolerance(pivotTolerance);

  }

  public double getTicks(WPI_TalonSRX motor) {
    return motor.getSelectedSensorPosition();
  }

  public void controlOverride(boolean controlState) {
    override = controlState;
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

  public double getLeftRPM() {
    return ((leftFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }

  public double getRightRPM() {
    return ((rightFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }

  public double getAverageRPM() {
    return ((getLeftRPM() + getRightRPM())/2.0);
  } 

  public double getAverageTangentialVelocity() {
    return ((getLeftVelocity() + getRightVelocity()) / 2.0);
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
    leftFlywheel.set((leftFlywheelFF.calculate(velocity))/12.0 + leftFlywheelPID.calculate(getLeftVelocity(), velocity)); // DIVIDE BY THE VOLTAGE!!!
    rightFlywheel.set((rightFlywheelFF.calculate(velocity))/12.0 + rightFlywheelPID.calculate(getRightVelocity(), velocity)); // DIVIDE BY THE VOLTAGE!!!
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

  public void setPivotAngle(double angle) {
    this.angle = angle; // We want it to always "brake" with constantly running feedforward!
    pivotPID.setSetpoint(this.angle);
  }

  public boolean atSetpoint() {
    return pivotPID.atSetpoint();
    // return (this.angle == getPivotPosition()); // We might want to revisit this one
  }

  public boolean isForwardLimitClosed() {
    if(pivot.isFwdLimitSwitchClosed() == 1) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean isReverseLimitClosed() {
    if(pivot.isRevLimitSwitchClosed() == 1) {
      return true;
    }
    else {
      return false;
    }
  }

  public double getPivotPosition(){
    return (((encoderConst*pivot.getSelectedSensorPosition(0)/4096.0)*360.0) + offset); // OFFSETTED is angle here. The negative const might not be needed
  }

  public double getPivotPositionNotOffset() {
    return (((encoderConst*pivot.getSelectedSensorPosition(0)/4096.0)*360.0));
  }

  public double getRawPivotPosition() {
    return ((encoderConst*pivot.getSelectedSensorPosition(0)) - ((offset/360.0)*4096.0));
  }

  public double getRawPivotPositionNotOffset() {
    return encoderConst*pivot.getSelectedSensorPosition(0);
  }

  public void setPivotPower(double power) {
    pivot.set(power); // This is what was tested March 1st
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
    SmartDashboard.putNumber("Left Flywheel RPM", getLeftRPM());
    SmartDashboard.putNumber("Left Flywheel Power", getLeftFlywheelPower());
    SmartDashboard.putNumber("Left Flywheel Ticks", getTicks(leftFlywheel));

    SmartDashboard.putNumber("Right Flywheel Velocity", getRightVelocity());
    SmartDashboard.putNumber("Right Flywheel RPM", getRightRPM());
    SmartDashboard.putNumber("Right Flywheel Power", getRightFlywheelPower());
    SmartDashboard.putNumber("Right Flywheel Ticks", getTicks(rightFlywheel));

    SmartDashboard.putNumber("Average Velocity", getAverageTangentialVelocity());
    SmartDashboard.putNumber("Average RPM", getAverageRPM());
    SmartDashboard.putNumber("Average Current", getFlywheelCurrent());
    
    SmartDashboard.putNumber("Roller Ticks", getRollerTicks());
    SmartDashboard.putNumber("Roller Power", getRollerPower());

    SmartDashboard.putNumber("Pivot Angle", getPivotPosition());
    SmartDashboard.putNumber("Pivot Angle with No Offset", getPivotPositionNotOffset()); // Use this to find the offset angle at horizontal power
    SmartDashboard.putNumber("Raw Pivot Angle with No Offset", getRawPivotPositionNotOffset()); 
    SmartDashboard.putNumber("Pivot Power", getPivotPower());

    SmartDashboard.putNumber("Pivot P", pivotTestInputPIDP.getDouble(Constants.pivotPIDConsts.pidP));
    SmartDashboard.putNumber("Pivot I", pivotTestInputPIDI.getDouble(Constants.pivotPIDConsts.pidI));

    // A return of 'true' means that the limit switch is active
    SmartDashboard.putBoolean("Forward Limit Switch: ", isForwardLimitClosed());
    SmartDashboard.putBoolean("Reverse Limit Switch: ", isReverseLimitClosed());


    if (pivot.isRevLimitSwitchClosed() == 0) {
      pivot.setSelectedSensorPosition(0, 0, 10);
    }
    /*
    if (RobotContainer.getJoy4().getRawButton(9)) {
      setFlywheelPower(1.0);
    } else if (RobotContainer.getJoy4().getRawButton(10)) {
      setFlywheelPower(-0.2); // Intake
    } else {
      setFlywheelPower(0.0);
    }
    */
    /*if (Math.abs(RobotContainer.getJoy4().getY()) >= 0.1) {
      setPivotPower(RobotContainer.getJoy4().getY());
    }
    else { */
    if (!override) {
      angleTimer.reset();
      if (angleTimer.get() >= angleTime) {
        if (RobotContainer.getJoy4().getRawButton(11)) {
          angle -= 5.0;
          setPivotAngle(angle);
          angleTimer.reset();
        } else if (RobotContainer.getJoy4().getRawButton(12)) {
          angle += 5.0;
          setPivotAngle(angle);
          angleTimer.reset();
        }
      }
      double ffCos = Math.cos(Math.toRadians(getPivotPosition()));
      SmartDashboard.putNumber("Pivot FF", ffCos*maxHorizontalPower);
      pivot.set(pivotPID.calculate(getPivotPosition()) + ffCos*maxHorizontalPower);
      if (RobotContainer.getJoy4().getRawButton(7)) {
        setRollerPower(1.0); //Intake
      } else if (RobotContainer.getJoy4().getRawButton(8)) {
        setRollerPower(-1.0);
      } else {
        setRollerPower(0.0);
      }
      // setRollerPower(RobotContainer.getJoy4().getY());
      // setPivotPower(RobotContainer.getJoy4().getY());
      setFlywheelPower(RobotContainer.getJoy4().getY());
    }

    // These methods just set the PID controller's constants so that we can tune them if needed
    leftFlywheelPID.setPID(leftFlywheelTestInputPIDP.getDouble(Constants.leftFlywheelPIDConsts.pidP), leftFlywheelTestInputPIDI.getDouble(Constants.leftFlywheelPIDConsts.pidI), leftFlywheelTestInputPIDD.getDouble(Constants.leftFlywheelPIDConsts.pidD));
    rightFlywheelPID.setPID(rightFlywheelTestInputPIDP.getDouble(Constants.rightFlywheelPIDConsts.pidP), rightFlywheelTestInputPIDI.getDouble(Constants.rightFlywheelPIDConsts.pidI), rightFlywheelTestInputPIDD.getDouble(Constants.rightFlywheelPIDConsts.pidD));
    pivotPID.setPID(pivotTestInputPIDP.getDouble(Constants.pivotPIDConsts.pidP), pivotTestInputPIDI.getDouble(Constants.pivotPIDConsts.pidI), pivotTestInputPIDD.getDouble(Constants.pivotPIDConsts.pidD));

  }

}