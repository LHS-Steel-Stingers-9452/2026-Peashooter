package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class VisionPros extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;

    private static final String LEFT_LL = "limelight-left";
    private Pose2d visionPose = new Pose2d();
     InterpolatingDoubleTreeMap tyToHoodAngleMap = new InterpolatingDoubleTreeMap();


    private int[] acceptedTags = {2, 3, 4, 5, 8, 9, 10, 11};


    public VisionPros(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        tyToHoodAngleMap.put(0.0, 1.746);
    }

    @Override
    public void periodic() {
        // Feed vision measurements into the drivetrain's pose estimator
        // System.out.println("periodic is running");
    
        processLimelight(LEFT_LL);
    }

    private void processLimelight(String limelightName) {

        // // Get the full Limelight results
        //LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        // // If no valid target, skip
        // if (!results.valid) return;

        var driveState = drivetrain.getState();
        double headingDog = driveState.Pose.getRotation().getDegrees(); 
        LimelightHelpers.SetRobotOrientation("limelight-left", headingDog, 0, 0, 0, 0, 0); 

       
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        
        if (shouldAcceptPoseEstimate(poseEstimate, driveState.Pose)) {
             drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.004, 0.004, 99999999));
             drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }
    }

    private boolean shouldAcceptPoseEstimate(LimelightHelpers.PoseEstimate estimate, Pose2d currentPose) {

         var distance = estimate.pose.getTranslation().getDistance(currentPose.getTranslation());

         if (estimate.rawFiducials.length > 0 && estimate.rawFiducials[0].ambiguity > 0.65) {
            // SmartDashboard.putNumber("pose ambiguity", estimate.rawFiducials[0].ambiguity);
            return false;
            
         }
         SmartDashboard.putNumber("pose distance", distance);

        if (distance > 4 && DriverStation.isEnabled()) {
            return false;
        }
        return LimelightHelpers.validPoseEstimate(estimate);

    }
 
    @Logged
    public Pose2d getpPose2d() {
        return visionPose;
    }

    public double getHoodAngleFromTy() {
        var ty = LimelightHelpers.getTY("limelight-left");
        if (ty == 0) {
            return 0;
        }
        return tyToHoodAngleMap.get(ty);
    }

    }