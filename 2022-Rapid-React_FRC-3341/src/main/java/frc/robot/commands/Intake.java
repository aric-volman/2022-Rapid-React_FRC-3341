
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.InfraredSensor;

public class Intake extends CommandBase {
  private BallHandler ballHandler;
  private InfraredSensor infrared;
  private Timer timeout = new Timer();
  /** Creates a new Intake. */
  public Intake(BallHandler bh, InfraredSensor ir) {
    ballHandler = bh;
    infrared = ir;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballHandler, infrared);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeout.start();
    timeout.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ballHandler.isForwardLimitClosed()){
      ballHandler.setPivotPower(0.1); // Motor goes clockwise, set this negative??
    } else {
      ballHandler.setPivotPower(0.0);
      ballHandler.setFlywheelConstantVelocity(0.75); // Need to determine the correct intake velocity
      ballHandler.setRollerPower(0.15); // Roller goes clockwise, set this as negative
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
    return (infrared.get() | timeout.get() >= 3.0);
  }
}