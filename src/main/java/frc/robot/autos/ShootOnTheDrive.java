package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class ShootOnTheDrive extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;
  private final Physics physics;

  private final Translation2d redTarget;
  private final Translation2d blueTarget;

  public ShootOnTheDrive(
      CommandSwerveDrivetrain drivetrain,
      Shooter shooter,
      Physics physics,
      Translation2d redTarget,
      Translation2d blueTarget) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.physics = physics;
    this.redTarget = redTarget;
    this.blueTarget = blueTarget;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    // Select target based on alliance
    Translation2d target = redTarget;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      target = blueTarget;
    }

    Pose2d pose = drivetrain.getState().Pose;
    var speeds = drivetrain.getState().Speeds;
    double shotTime = 1.2;
    Translation2d velocityOffset =
        new Translation2d(speeds.vxMetersPerSecond * shotTime, speeds.vyMetersPerSecond * shotTime);
    Translation2d adjustedTarget = target.minus(velocityOffset);
    double distance = pose.getTranslation().getDistance(adjustedTarget);
    double targetVelocity = physics.setVelocity(distance);
    shooter.setVelocityDirect(targetVelocity);

    Translation2d direction = target.minus(pose.getTranslation());
    double error = direction.getAngle().getRadians() - pose.getRotation().getRadians();
    double kP = 3.0;
    double targetingAngularVelocity = error * kP;

    // drivetrain.applyRequest(() ->
    //     drivetrain.getDriveRequest()
    //         .withVelocityX(-joystick.getLeftY() * maxSpeed)
    //         .withVelocityY(-joystick.getLeftX() * maxSpeed)
    //         .withRotationalRate(targetingAngularVelocity)
    // );
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
