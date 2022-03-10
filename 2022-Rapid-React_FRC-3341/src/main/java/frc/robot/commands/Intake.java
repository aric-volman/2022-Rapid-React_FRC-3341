
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.InfraredSensor;

public class Intake extends CommandBase {
  private BallHandler ballHandler;
  private InfraredSensor infrared;
  private Timer timeout = new Timer();
  private double notThereYetTime = 5.0; // We really need to adjust this
  /** Creates a new Intake. 
   * @param bh - The BallHandler subsystem
   * @param ir - The Infrared Sensor subsystem
  */
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
      ballHandler.setPivotAngle(-Constants.angularOffset); // We'd prefer not to set the power directly
    } else {
      ballHandler.setFlywheelPower(-1.0); // Negative should mean "suck", this might be too strong
      ballHandler.setRollerPower(-1.0); // Negative should mean "suck"
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
    return (infrared.get() | timeout.get() >= notThereYetTime);
  }
}