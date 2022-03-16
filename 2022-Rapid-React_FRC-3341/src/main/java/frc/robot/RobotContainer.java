// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Ultrasonic;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static BallHandler ballHandler;
  private static MaxbotixUltrasonicSensor ultrasonicSensor;
  private static InfraredSensor infrared;
  private static Limelight limelight;
  private static RotatePID rotatePID;
  private static DriveTrain _DriveTrain;
  private final TankDrive _tankDrive;

  private static Arm1 frontLeftSub;
  private static Arm1 frontRightSub;
  private static Arm1 backLeftSub;
  private static Arm1 backRightSub;
  private static ArmMoveTeleop frontLeftCom;
  private static ArmMoveTeleop frontRightCom;
  private static ArmMoveTeleop backLeftCom;
  private static ArmMoveTeleop backRightCom;
  private static FourArmMoveTeleop fourArmMoveTeleop;
  //private static Arm arm;
  private static ArmExtend extend;
  private static ArmExtendSeq extendSeq;
  private static setDefaultCommand armDefault;
  private static AutoPath autoPath;

  public static Joystick joy1;
  public static Joystick joy2;
  public static Joystick joy3;
  public static Joystick joy4;

  public static JoystickButton but5;
  public static JoystickButton but6;
  public static JoystickButton but3;
  public static JoystickButton but4;
  public static JoystickButton but2;
  public static JoystickButton but12;

  private static Boolean isDriving;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    joy1 = new Joystick(0);
    joy2 = new Joystick(1);
    joy3 = new Joystick(2);
    joy4 = new Joystick(3);

    /*but5 = new JoystickButton(joy3, 5);
    but6 = new JoystickButton(joy3, 6);
    but3 = new JoystickButton(joy3, 3);
    but4 = new JoystickButton(joy3, 4);
    but2 = new JoystickButton(joy3, 2);*/

    but12 = new JoystickButton(joy1, 12);
    
    isDriving = true;

    ultrasonicSensor = new MaxbotixUltrasonicSensor(Constants.I2CAddresses.MaxbotixUltrasonicSensor);
    infrared = new InfraredSensor();
    limelight = new Limelight();
    ballHandler = new BallHandler();
    //ultrasonicSensor = new Ultrasonic();
    _DriveTrain = new DriveTrain();
    _tankDrive = new TankDrive(_DriveTrain, joy1, joy2);
    _DriveTrain.setDefaultCommand(_tankDrive);
    autoPath = new AutoPath();

    //arm = new Arm();
    frontLeftSub = new Arm1(Constants.ArmPorts.FrontLeftArmExt, Constants.ArmPorts.FrontLeftArmRot, 0, "FrontLeft");
    frontRightSub = new Arm1(Constants.ArmPorts.FrontRightArmExt, Constants.ArmPorts.FrontRightArmRot, 1, "FrontRight");
    backLeftSub = new Arm1(Constants.ArmPorts.BackLeftArmExt, Constants.ArmPorts.BackLeftArmRot, 2, "BackLeft");
    backRightSub = new Arm1(Constants.ArmPorts.BackRightArmExt, Constants.ArmPorts.BackRightArmRot, 3, "BackRight");
    frontLeftCom = new ArmMoveTeleop(frontLeftSub, joy1);
    frontRightCom = new ArmMoveTeleop(frontRightSub, joy2);
    backLeftCom = new ArmMoveTeleop(backLeftSub, joy3);
    backRightCom = new ArmMoveTeleop(backRightSub, joy4);
    frontLeftSub.setDefaultCommand(frontLeftCom);
    frontRightSub.setDefaultCommand(frontRightCom);
    backLeftSub.setDefaultCommand(backLeftCom);
    backRightSub.setDefaultCommand(backRightCom);
    //armDefault = new setDefaultCommand(frontLeftSub, frontRightSub, backLeftSub, backRightSub, _DriveTrain);
    //fourArmMoveTeleop = new FourArmMoveTeleop(frontLeftSub, frontRightSub, backLeftSub, backRightSub);

    //rotatePID = new RotatePID(20);
    //extend = new ArmExtend(5);
    extendSeq = new ArmExtendSeq();

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //but12.toggleWhenPressed(armDefault);
    /*
     but5.whenPressed(frontLeftCom);
     but6.whenPressed(frontRightCom);
     but3.whenPressed(backLeftCom);
     but4.whenPressed(backRightCom);
     but2.whenPressed(fourArmMoveTeleop);

     but5.cancelWhenPressed(frontRightCom);
     but5.cancelWhenPressed(backLeftCom);
     but5.cancelWhenPressed(backRightCom);
     but5.cancelWhenPressed(fourArmMoveTeleop);
     but6.cancelWhenPressed(frontLeftCom);
     but6.cancelWhenPressed(backLeftCom);
     but6.cancelWhenPressed(backRightCom);
     but6.cancelWhenPressed(fourArmMoveTeleop);
     but3.cancelWhenPressed(frontLeftCom);
     but3.cancelWhenPressed(frontRightCom);
     but3.cancelWhenPressed(backRightCom);
     but3.cancelWhenPressed(fourArmMoveTeleop);
     but4.cancelWhenPressed(frontLeftCom);
     but4.cancelWhenPressed(frontRightCom);
     but4.cancelWhenPressed(backLeftCom);
     but4.cancelWhenPressed(fourArmMoveTeleop);
     but2.cancelWhenPressed(frontLeftCom);
     but2.cancelWhenPressed(frontRightCom);
     but2.cancelWhenPressed(backLeftCom);
     but2.cancelWhenPressed(backRightCom);

     activateHolding();
     */
  }

  public static void activateHolding() {
    frontLeftSub.setDefaultCommand(new DefaultExtend(frontLeftSub, -0.2));
    frontRightSub.setDefaultCommand(new DefaultExtend(frontRightSub, -0.2));
    backLeftSub.setDefaultCommand(new DefaultExtend(backLeftSub, -0.2));
    backRightSub.setDefaultCommand(new DefaultExtend(backRightSub, -0.2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return extendSeq;
    return autoPath;
  }

  public static DriveTrain getDriveTrain() {
    return _DriveTrain;
  }

  public static Limelight getLimelight(){
    return limelight;
  }

 /* public static Arm getArm(){
    return arm;
  }*/

  public static Joystick getJoy1(){
    return joy1;
  }

  public static Joystick getJoy2(){
    return joy2;
  }

  public static Joystick getJoy3() {
    return joy3;
  }

  public static Joystick getJoy4() {
    return joy4;
  }

  public static MaxbotixUltrasonicSensor getUltrasonic() {
    return ultrasonicSensor;
  }

  public static BallHandler getBallHandler() {
    return ballHandler;
  }

  public static Boolean getIsDriving() {
    return isDriving;
  }

  public static void switchIsDriving() {
    isDriving = !isDriving;
  }

}
