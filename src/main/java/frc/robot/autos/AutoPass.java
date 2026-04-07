package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;

@Logged
public class AutoPass extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;
  private final Physics physics;
  // private final Hood hood;
  private final SwerveRequest.FieldCentric drive;
  private final CommandPS5Controller joystick;
  private final Translation2d redTarget;
  private final Translation2d blueTarget;

  public AutoPass(
      CommandSwerveDrivetrain drivetrain,
      Shooter shooter,
      Physics physics,
      // Hood hood,
      SwerveRequest.FieldCentric drive,
      CommandPS5Controller joystick,
      Translation2d redTarget,
      Translation2d blueTarget) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.physics = physics;
    // this.hood = hood;
    this.redTarget = redTarget;
    this.blueTarget = blueTarget;
    this.joystick = joystick;
    this.drive = drive;
    addRequirements(shooter);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // Select target based on alliance
    Translation2d target = redTarget;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      target = blueTarget;
    }

    //Pulling Speeds of Chassis
    Pose2d pose = drivetrain.getState().Pose;
    var speeds = drivetrain.getState().Speeds;
    var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, pose.getRotation());
    
    //robot velocity
    Translation2d robotVelocity = new Translation2d(
        fieldSpeeds.vxMetersPerSecond,
        fieldSpeeds.vyMetersPerSecond);

    // 0. latency compensation
    double latency = 0.1; // tune this, likely 0.1-0.2s
    Translation2d futurePos = pose.getTranslation().plus(robotVelocity.times(latency));

    // 1. Compute vector from robot to real target
    Translation2d toTarget = target.minus(futurePos);
    double realDistance = toTarget.getNorm();
    Translation2d targetDirection = toTarget.div(realDistance); // unit vector

    // 2. Look up horizontal velocity from tuned table at real distance (v = d / tof)
    double shotTime = physics.getShotTime(realDistance);
    // double horizontalVelocity = realDistance / shotTime; PUT PACK IF NOT WORK
    double shooterSpeed = physics.setVelocity(realDistance); //CHAT
    Translation2d targetVelocityVector = targetDirection.times(shooterSpeed); //CHAT

    // 3. Build the ideal stationary shot velocity vector
    // Translation2d targetVelocityVector = targetDirection.times(horizontalVelocity); PUT BACK IF NOT WORK

    // 4. Subtract robot velocity to get the required exit vector
    Translation2d shotVector = targetVelocityVector.minus(robotVelocity);

    // 5. Derive aim angle from the corrected shot vector
    Rotation2d aimAngle = shotVector.getAngle();
    double error = aimAngle.minus(pose.getRotation()).getRadians();

    // 6. Set shooter velocity based on real distance
    double targetVelocity = physics.setVelocity(realDistance);

    // double targetHood = physics.getHoodAngle(realDistance);

    shooter.setVelocityDirect(targetVelocity);
    // hood.setPosition(targetHood);
    // 7. Drive with corrected rotational rate
    double kP = 3.0;
    double targetingAngularVelocity = error * kP;

    var MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    drivetrain.setControl(
        drive
            .withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withRotationalRate(targetingAngularVelocity));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setVelocity(0);
    // hood.setPosition(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}