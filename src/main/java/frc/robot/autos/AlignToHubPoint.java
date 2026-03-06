package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToHubPoint extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private Translation2d driveTarget;
    private final Translation2d[] possibleTargets;
    private final Translation2d hubPosition;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    private final PIDController xController = new PIDController(2.0, 0, 0);
    private final PIDController yController = new PIDController(2.0, 0, 0);
    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            3.0, 0, 0,
            new TrapezoidProfile.Constraints(
                6.0,   // max rad/s
                12.0   // max rad/s²
            )
        );

    private final HolonomicDriveController controller =
        new HolonomicDriveController(xController, yController, thetaController);


    private final double maxSpeed = 3.0;   // m/s
    private final double maxOmega = 6.0;   // rad/s
    private final double positionTolerance = 0.08; // meters
    private final double rotationTolerance = Math.toRadians(2); // radians

    public AlignToHubPoint(CommandSwerveDrivetrain drivetrain, Translation2d driveTarget, Translation2d[] possibleTargets, Translation2d hubPosition) {
        this.drivetrain = drivetrain;
        this.driveTarget = driveTarget;
        this.possibleTargets = possibleTargets;
        this.hubPosition = hubPosition;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    Pose2d robotPose = drivetrain.getState().Pose;

    Translation2d closest = possibleTargets[0];
    double bestDistance =
        closest.getDistance(robotPose.getTranslation());

    for (Translation2d target : possibleTargets) {

        double distance =
            target.getDistance(robotPose.getTranslation());

        if (distance < bestDistance) {
            bestDistance = distance;
            closest = target;
        }
    }

    driveTarget = closest;
}
    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;

        // Compute heading toward hub
        Translation2d direction = hubPosition.minus(currentPose.getTranslation());
        Rotation2d desiredHeading = direction.getAngle();

        // Goal pose at the target coordinates, facing the hub
        Pose2d goalPose = new Pose2d(driveTarget, desiredHeading);

        // Calculate chassis speeds to drive there
        ChassisSpeeds speeds = controller.calculate(currentPose, goalPose, 0.0, desiredHeading);

        // Clamp outputs
        double vx = MathUtil.clamp(speeds.vxMetersPerSecond, -maxSpeed, maxSpeed);
        double vy = MathUtil.clamp(speeds.vyMetersPerSecond, -maxSpeed, maxSpeed);
        double omega = MathUtil.clamp(speeds.omegaRadiansPerSecond, -maxOmega, maxOmega);

        drivetrain.setControl(drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
    }

    @Override
    public boolean isFinished() {

    Pose2d current = drivetrain.getState().Pose;

    Rotation2d desiredHeading =
        hubPosition.minus(current.getTranslation()).getAngle();

    boolean positionDone =
        current.getTranslation().getDistance(driveTarget)
        < positionTolerance;

    boolean rotationDone =
        Math.abs(current.getRotation()
        .minus(desiredHeading)
        .getRadians())
        < rotationTolerance;

    return positionDone && rotationDone;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}