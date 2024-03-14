package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import java.util.function.DoubleSupplier;

public class ManualWrist extends Command {

  Wrist s_Wrist;
  DoubleSupplier power;

  public ManualWrist(Wrist s_Wrist, DoubleSupplier power) {
    this.s_Wrist = s_Wrist;
    this.power = power;
    addRequirements(s_Wrist);
  }

  @Override
  public void execute() {
    s_Wrist.set(power.getAsDouble()); // + 0.01
  }

  @Override
  public void end(boolean interrupted) {
    s_Wrist.set(0);
  }
}
