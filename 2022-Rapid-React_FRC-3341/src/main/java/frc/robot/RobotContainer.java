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

  private static Joystick joystick;
  private static JoystickButton shootbutton;
  private static JoystickButton intakebutton;
  private static JoystickButton flywheelbutton;
  private static JoystickButton rollerbutton;
  private static JoystickButton setanglebutton;
  private static JoystickButton resetanglebutton;

  private double flywheelvelocity = 1.0;

  private double angle = 30;

  private static final BallHandler ballHandler = new BallHandler();

  private MaxbotixUltrasonicSensor ultrasonicSensor = new MaxbotixUltrasonicSensor(Constants.I2CAddresses.MaxbotixUltrasonicSensor);
  private InfraredSensor infrared = new InfraredSensor();

  // The robot's subsystems and commands are defined here...

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    joystick = new Joystick(Constants.JoystickPorts.JoystickPort1);
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

    shootbutton = new JoystickButton(joystick, 1);
    shootbutton.whenPressed(new EncoderShootAtAngle(flywheelvelocity, angle, ballHandler));
    
    intakebutton = new JoystickButton(joystick, 2);
    intakebutton.whenPressed(new Intake(ballHandler, infrared));

    flywheelbutton = new JoystickButton(joystick, 3);
    flywheelbutton.toggleWhenPressed(new ManualFlywheel(flywheelvelocity, ballHandler), false);
    
    rollerbutton = new JoystickButton(joystick, 4);
    //rollerbutton.toggleWhenPressed(new ManualRoller(rollerpower));
    rollerbutton.whenPressed(new SetAnglePID(10.0, ballHandler), false);
    
    setanglebutton = new JoystickButton(joystick, 5);
    setanglebutton.whenPressed(new SetAnglePID(angle, ballHandler), false); // Changes it to non-interruptable
    
    resetanglebutton = new JoystickButton(joystick, 6);
    //resetanglebutton.toggleWhenPressed(new SetAngle(0));
    resetanglebutton.whenPressed(new SetAnglePID(0.0, ballHandler), false);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public static Joystick getJoystick() {
    return joystick;
  }

  public static BallHandler returnBallHandler() { // Needed for EncoderShoot
    return ballHandler;
  }
 
}
