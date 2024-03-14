package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;
import java.util.function.DoubleSupplier;

public class ManualLift extends Command {
  Lift s_Lift;
  DoubleSupplier power;

  public ManualLift(Lift s_Lift, DoubleSupplier power) {
    this.s_Lift = s_Lift;
    this.power = power;
    addRequirements(s_Lift);
  }

  @Override
  public void execute() {
    s_Lift.run(power);
  }
}
