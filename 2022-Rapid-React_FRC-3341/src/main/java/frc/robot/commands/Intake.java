
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.InfraredSensor;
import frc.robot.subsystems.MaxbotixUltrasonicSensor;

public class Intake extends CommandBase {
  private BallHandler ballHandler;
  private InfraredSensor infrared;
  private MaxbotixUltrasonicSensor ultrasonic;
  /** Creates a new Intake. */
  public Intake(BallHandler b, InfraredSensor ir, MaxbotixUltrasonicSensor mb) {
    ballHandler = b;
    infrared = ir;
    ultrasonic = mb;
    addRequirements(ballHandler, infrared);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ballHandler.isForwardLimitClosed()){
      ballHandler.setPivotPower(0.1); //motor goes clockwise, set this negative??
    } else {
      ballHandler.setPivotPower(0.0);
      ballHandler.setFlywheelConstantVelocity(0.75);
      ballHandler.setRollerPower(0.15); //roller goes clockwise, set this as negative
      } 
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballHandler.setFlywheelPower(0.0);
    ballHandler.setRollerPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return infrared.get();
    return (ultrasonic.getDistance() <= 0.3);
  }
}