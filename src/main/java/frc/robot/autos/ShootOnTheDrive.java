// package frc.robot.autos;

// import static edu.wpi.first.units.Units.MetersPerSecond;

// import java.lang.annotation.Target;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.epilogue.Epilogue;
// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Shooter;

// @Logged
// public class ShootOnTheDrive extends Command {

//   private final CommandSwerveDrivetrain drivetrain;
//   private final Shooter shooter;
//   private final Physics physics;
//   private final SwerveRequest.FieldCentric drive;
//   private final CommandPS5Controller joystick;

//   private final Translation2d redTarget;
//   private final Translation2d blueTarget;

  
//   private Translation2d logPose;

//   public ShootOnTheDrive(
//       CommandSwerveDrivetrain drivetrain,
//       Shooter shooter,
//       Physics physics,
//        SwerveRequest.FieldCentric drive, 
//        CommandPS5Controller joystick,      
//       Translation2d redTarget,
//       Translation2d blueTarget) {
//     this.drivetrain = drivetrain;
//     this.shooter = shooter;
//     this.physics = physics;
//     this.redTarget = redTarget;
//     this.blueTarget = blueTarget;
//     this.joystick = joystick;
//     this.drive = drive;
//     addRequirements(shooter);
//     addRequirements(drivetrain);
//   }

//   @Override
//   public void execute() {
//     // Select target based on alliance
//     Translation2d target = redTarget;
//     var alliance = DriverStation.getAlliance();
//     if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
//       target = blueTarget;
//     }

//     Pose2d pose = drivetrain.getState().Pose;
//     var speeds = drivetrain.getState().Speeds;
//     var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds,pose.getRotation());

//     var realDistance = pose.getTranslation().getDistance(target);
//     double shotTime = physics.getShotTime(realDistance);
//     Translation2d aimOffset =
//         new Translation2d(fieldSpeeds.vxMetersPerSecond * shotTime, fieldSpeeds.vyMetersPerSecond * shotTime);
//     Translation2d adjustedTarget = target.plus(aimOffset);
//     // logPose = adjustedTarget;
//     SmartDashboard.putNumberArray("adj target", new double[]{adjustedTarget.getX(), adjustedTarget.getY()});
    
//     double distance = pose.getTranslation().getDistance(adjustedTarget);
//     double targetVelocity = physics.setVelocity(distance);
//     shooter.setVelocityDirect(targetVelocity);

//     Translation2d direction = adjustedTarget.minus(pose.getTranslation());
//     // double error = direction.getAngle().getRadians() - pose.getRotation().getRadians();
//      var error = direction.getAngle().minus(pose.getRotation()).getRadians();

//     double kP = 3.0; 
//     double targetingAngularVelocity = error * kP;

//     var MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    
//     drivetrain.setControl(
//             drive
//               .withVelocityX(
//                   -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
//               .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
//               .withRotationalRate(
//                   targetingAngularVelocity)); // Drive counterclockwise with negative X (left)


//     // drivetrain.applyRequest(() ->
//     //     drivetrain.getDriveRequest()
//     //         .withVelocityX(-joystick.getLeftY() * maxSpeed)
//     //         .withVelocityY(-joystick.getLeftX() * maxSpeed)
//     //         .withRotationalRate(targetingAngularVelocity)
//     // );
//   }

//   @Override
//   public void end(boolean interrupted) {
//     shooter.setVelocity(0);
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
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

@Logged
public class ShootOnTheDrive extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;
  private final Physics physics;
  private final SwerveRequest.FieldCentric drive;
  private final CommandPS5Controller joystick;
  private final Translation2d redTarget;
  private final Translation2d blueTarget;

  public ShootOnTheDrive(
      CommandSwerveDrivetrain drivetrain,
      Shooter shooter,
      Physics physics,
      SwerveRequest.FieldCentric drive,
      CommandPS5Controller joystick,
      Translation2d redTarget,
      Translation2d blueTarget) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.physics = physics;
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
    double latency = 0.2; // tune this, likely 0.1-0.2s
    Translation2d futurePos = pose.getTranslation().plus(robotVelocity.times(latency));

    // 1. Compute vector from robot to real target
    Translation2d toTarget = target.minus(futurePos);
    double realDistance = toTarget.getNorm();
    Translation2d targetDirection = toTarget.div(realDistance); // unit vector

    // 2. Look up horizontal velocity from tuned table at real distance (v = d / tof)
    double shotTime = physics.getShotTime(realDistance);
    double horizontalVelocity = realDistance / shotTime;

    // 3. Build the ideal stationary shot velocity vector
    Translation2d targetVelocityVector = targetDirection.times(horizontalVelocity);

    // 4. Subtract robot velocity to get the required exit vector
    Translation2d shotVector = targetVelocityVector.minus(robotVelocity);

    // 5. Derive aim angle from the corrected shot vector
    Rotation2d aimAngle = shotVector.getAngle();
    double error = aimAngle.minus(pose.getRotation()).getRadians();

    // 6. Set shooter velocity based on real distance
    double targetVelocity = physics.setVelocity(realDistance);
    shooter.setVelocityDirect(targetVelocity);

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
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}