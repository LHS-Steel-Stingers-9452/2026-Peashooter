package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.autos.Physics;

public class AutoShoot extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final Physics physics;

    private final Translation2d redTarget;
    private final Translation2d blueTarget;

    // Optional smoothing
    // private double filteredVelocity = 0;

    public AutoShoot(
        CommandSwerveDrivetrain drivetrain,
        Shooter shooter,
        Physics physics,
        Translation2d redTarget,
        Translation2d blueTarget
    ) {
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
            
        // var currentPose = drivetrain.getState().Pose;
        // var direction = target.minus(currentPose.getTranslation());
        // Get robot pose
        Pose2d pose = drivetrain.getState().Pose;

        // Calculate distance
        var distance = pose.getTranslation().getDistance(target);

        // Get velocity from physics
        double targetVelocity = physics.setVelocity(distance);

        // Set shooter velocity ONLY
        shooter.setVelocityDirect(targetVelocity);
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