package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax spin = new CANSparkMax(9, MotorType.kBrushless);

  public Intake() {
    spin.setIdleMode(IdleMode.kBrake);
  }
  ;

  public void setPower(double power) {
    spin.set(power);
  }

  public void stop() {
    spin.set(0);
  }
}
