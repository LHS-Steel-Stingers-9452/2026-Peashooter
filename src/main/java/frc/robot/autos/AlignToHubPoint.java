// package frc.robot.autos;

// import edu.wpi.first.math.controller.HolonomicDriveController;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// public class AlignToHubPoint extends Command {

//     private final CommandSwerveDrivetrain drivetrain;
//     private Pose2d targetPose;

//     private final PIDController xController = new PIDController(3.0, 0, 0);
//     private final PIDController yController = new PIDController(3.0, 0, 0);
    
//     private final SwerveRequest.FieldCentric drive =
//     new SwerveRequest.FieldCentric();

//     private final ProfiledPIDController thetaController =
//         new ProfiledPIDController(
//             4.0, 0, 0,
//             new TrapezoidProfile.Constraints(
//                 6.28,   // max angular velocity (rad/s)
//                 12.56   // max angular accel (rad/s²)
//             )
//         );

//     private final HolonomicDriveController controller =
//         new HolonomicDriveController(
//             xController,
//             yController,
//             thetaController
//         );

//     public AlignToHubPoint(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
//         this.drivetrain = drivetrain;
//         this.targetPose = targetPose;

//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         addRequirements(drivetrain);
//     }

//     @Override
//     public void initialize() {
//         targetPose = allianceFlip(targetPose);
//     }

//     @Override
//     public void execute() {

//         Pose2d currentPose = drivetrain.getState().Pose;

//         ChassisSpeeds speeds = controller.calculate(
//             currentPose,
//             targetPose,
//             0.0,
//             targetPose.getRotation()
//         );

//         drivetrain.setControl(
//     drive
//         .withVelocityX(speeds.vxMetersPerSecond)
//         .withVelocityY(speeds.vyMetersPerSecond)
//         .withRotationalRate(speeds.omegaRadiansPerSecond)
// );
//     }

//     @Override
//     public boolean isFinished() {
//         Pose2d current = drivetrain.getState().Pose;

//         boolean positionDone =
//             current.getTranslation()
//                 .getDistance(targetPose.getTranslation()) < 0.05;

//         boolean rotationDone =
//             Math.abs(
//                 current.getRotation()
//                     .minus(targetPose.getRotation())
//                     .getDegrees()
//             ) < 2;

//         return positionDone && rotationDone;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.setControl(new SwerveRequest.Idle());
//     }

//     private Pose2d allianceFlip(Pose2d pose) {

//         if (DriverStation.getAlliance().isPresent() &&
//             DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {

//             double fieldLength = 16.54; // update if needed

//             return new Pose2d(
//                 fieldLength - pose.getX(),
//                 pose.getY(),
//                 pose.getRotation().rotateBy(
//                     edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180)
//                 )
//             );
//         }

//         return pose;
//     }
// }