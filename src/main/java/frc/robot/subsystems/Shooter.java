package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final CANSparkMax shootHigh = new CANSparkMax(19, MotorType.kBrushless);
  private final CANSparkMax shootLow = new CANSparkMax(18, MotorType.kBrushless);

  public Shooter() {
    shootHigh.setInverted(false);
    shootLow.setInverted(true);
  }

  public void shoot() {
    shootHigh.set(1);
    shootLow.set(1);
  }

  public void stop() {
    shootHigh.set(0);
    shootLow.set(0);
  }
}
