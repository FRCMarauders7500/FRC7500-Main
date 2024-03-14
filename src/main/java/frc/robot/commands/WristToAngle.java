package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class WristToAngle extends Command {
  Wrist s_Wrist;
  double angle;

  public WristToAngle(Wrist s_Wrist, double angle) {
    this.s_Wrist = s_Wrist;
    this.angle = angle;
    addRequirements(s_Wrist);
  }

  @Override
  public void initialize() {
    s_Wrist.setTarget(angle);
  }

  @Override
  public void execute() {
    double pwr = s_Wrist.getControllerOutput();
    s_Wrist.set(pwr);
  }

  @Override
  public void end(boolean interrupted) {
    s_Wrist.set(0);
  }

  // @Override
  // public boolean isFinished() {
  //   return s_Wrist.controllerAtSetPoint();
  // }
}
