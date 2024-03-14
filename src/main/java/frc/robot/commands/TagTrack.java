package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class TagTrack extends Command {
  Drive s_Drive;

  public TagTrack() {}

  @Override
  public void execute() {
    DriveCommands.joystickDrive(
        s_Drive,
        s_Drive::limelightRangeProportional,
        s_Drive::limelightY,
        s_Drive::limelightAimProportional);
  }

  @Override
  public void end(boolean interrupted) {
    s_Drive.stop();
  }
}
