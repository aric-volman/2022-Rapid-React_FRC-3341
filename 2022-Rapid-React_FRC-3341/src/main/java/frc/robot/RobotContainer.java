// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static Joystick leftDriveJoystick;
  private static Joystick rightDriveJoystick;
  private static Joystick ballHandlerJoystick;
  private static JoystickButton shootbutton;
  private static JoystickButton intakebutton;
  private static JoystickButton shootanglebutton;
  private static JoystickButton zeroanglebutton;
  // private static JoystickButton setanglebutton;
  // private static JoystickButton resetanglebutton;

  private double angle = 45; // Degrees from horizontal

  private static final BallHandler ballHandler = new BallHandler();
  private DriveTrain dt;
  private TankDrive td;

  private MaxbotixUltrasonicSensor ultrasonicSensor = new MaxbotixUltrasonicSensor(Constants.I2CAddresses.MaxbotixUltrasonicSensor);
  private InfraredSensor infrared = new InfraredSensor();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    ballHandlerJoystick = new Joystick(Constants.USBOrder.BallHandlerJoystickPort);
    // dt = new DriveTrain();
    // td = new TankDrive(dt, leftDriveJoystick, rightDriveJoystick);
    // dt.setDefaultCommand(td);
    // Configure the button bindings
    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Button Bindings -- a perpetual WIP
    
    intakebutton = new JoystickButton(ballHandlerJoystick, 1);
    intakebutton.whenPressed(new Intake(ballHandler, infrared));

    //shootbutton = new JoystickButton(ballHandlerJoystick, 2);
    //shootbutton.(new EncoderShoot(ballHandler)); // Velocity not used for now, shoots at 2200 RPM
/*
    shootanglebutton = new JoystickButton(ballHandlerJoystick, 5);
    shootanglebutton.toggleWhenPressed(new SetAnglePID(angle, ballHandler), false);
    
    zeroanglebutton = new JoystickButton(ballHandlerJoystick, 6);
    zeroanglebutton.whenPressed(new SetAnglePID(-Constants.angularOffset, ballHandler), false);
    
     Overriden by subsystem
    setanglebutton = new JoystickButton(joystick, 3);
    setanglebutton.whenPressed(new SetAnglePID(angle, ballHandler), false); // Changes it to non-interruptable
    
    resetanglebutton = new JoystickButton(joystick, 4);
    //resetanglebutton.toggleWhenPressed(new SetAngle(0));
    resetanglebutton.whenPressed(new SetAnglePID(0.0, ballHandler), false);
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public static Joystick getBallHandlerJoystick() {
    return ballHandlerJoystick;
  }
 
}
