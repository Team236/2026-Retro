package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class ZeroPose extends Command {
  private Swerve s_Swerve;
  private Pose2d pose2D;

  public ZeroPose(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    pose2D = new Pose2d(0, 0, new Rotation2d(0));
  }

  @Override
  public void initialize() {
    s_Swerve.resetPose(pose2D);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}