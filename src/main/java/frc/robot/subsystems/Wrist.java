package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Wrist extends SubsystemBase {
  private final CANSparkMax wristUp = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax wristDown = new CANSparkMax(11, MotorType.kBrushless);
  private PIDController angleController = new PIDController(.02, 0, 0);
  private double encZero;
  private RelativeEncoder enc = wristUp.getEncoder();

  public Wrist() {
    encZero = 0;
    enc.setPosition(encZero);
    wristDown.follow(wristUp);
    wristUp.setIdleMode(IdleMode.kBrake);
    wristDown.setIdleMode(IdleMode.kBrake);

    angleController.setTolerance(2);
  }

  public double getAngle() {
    return (enc.getPosition() - encZero)
        / Constants.WRIST_GEAR_RATIO
        * Constants.WRIST_TICKS_PER_ROTATION
        / 360.0
        / 2.52;
  }

  public void setTarget(double angle) {
    angleController.setSetpoint(angle);
  }

  public double getControllerOutput() {
    return MathUtil.clamp(angleController.calculate(getAngle()), -1, 1);
  }

  public boolean controllerAtSetPoint() {
    return angleController.atSetpoint();
  }

  public void set(double pwr) {
    wristUp.set(pwr);
  }

  public void manualSet(DoubleSupplier pwr) {
    wristUp.set(pwr.getAsDouble() * 0.5);
  }

  public void resetEncoder() {
    encZero = enc.getPosition();
  }
}
