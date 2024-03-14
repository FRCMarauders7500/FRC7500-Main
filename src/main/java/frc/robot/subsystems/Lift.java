package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Lift extends SubsystemBase {
  private final CANSparkMax lift1 = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax lift2 = new CANSparkMax(13, MotorType.kBrushless);

  public Lift() {
    lift2.setInverted(true);
  }

  public void run(DoubleSupplier power) {
    double pwr = power.getAsDouble();
    MathUtil.applyDeadband(pwr, .1);
    lift1.set(pwr);
    lift2.set(pwr);
  }
}
