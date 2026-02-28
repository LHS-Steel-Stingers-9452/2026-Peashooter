package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
     private final Pigeon2 pigeon;


    private int[] acceptedTags = {2, 3, 4, 5, 8, 9, 10, 11};


    public VisionPros(CommandSwerveDrivetrain drivetrain, Pigeon2 pigeon) {
        this.drivetrain = drivetrain;
        this.pigeon = pigeon;
        tyToHoodAngleMap.put(0.0, 1.746);
        LimelightHelpers.setCameraPose_RobotSpace( "limelight-left", 
     0.381, 
     0.1397, 
     0.5207, 
     0, 
     10, 
     0 );

     
        
    }

    @Override
    public void periodic() {
        // Feed vision measurements into the drivetrain's pose estimator
        // System.out.println("periodic is running");
    
        processLimelight(LEFT_LL);
        // System.out.println("getYaw():" + pigeon.getYaw().getValue());
        // System.out.println("getAngle():" + pigeon.getAngle());
        // System.out.println("getRotation3d():" + pigeon.getRotation3d().getAngle());
        // // System.out.println("getAbsoluteCompassHeadng():" + pigeon.getAbsoluteCompassHeadng().getValue());
        




    }
    
    private void processLimelight(String limelightName) {
        double rawGyroYaw = pigeon.getYaw().getValueAsDouble();
        // double correctedYaw = -rawGyroYaw;

        var driveState = drivetrain.getState();
        double headingDog = driveState.Pose.getRotation().getDegrees();
        double rawGyroRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();
         

        LimelightHelpers.SetRobotOrientation("limelight-left",
         rawGyroYaw, 
        0,
        10,
        0,
        0,
        0); 
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
        
       boolean doRejectUpdate = false;
       if (Math.abs(rawGyroRate) > 360) {
        doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
        drivetrain.setVisionMeasurementStdDevs(
        VecBuilder.fill(0.004, 0.004, 99999999)
        );
        drivetrain.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds
        );
        visionPose = mt2.pose;
        }

        // if (shouldAcceptPoseEstimate(mt2, driveState.Pose)) {
        //      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.004, 0.004, 99999999));
        //      drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        //      visionPose = mt2.pose;
        // }
        // System.out.println("Gyro Yaw: " + rawGyroYaw);
        // System.out.println("Estimator Rot: " + drivetrain.getState().Pose.getRotation().getDegrees());
        // System.out.println("Vision Rot: " + mt2.pose.getRotation().getDegrees());
    }

    // private boolean shouldAcceptPoseEstimate(LimelightHelpers.PoseEstimate estimate, Pose2d currentPose) {

    //      var distance = estimate.pose.getTranslation().getDistance(currentPose.getTranslation());

    //      if (estimate.rawFiducials.length > 0 && estimate.rawFiducials[0].ambiguity > 0.65) {
    //         // SmartDashboard.putNumber("pose ambiguity", estimate.rawFiducials[0].ambiguity);
    //         return false;
            
    //      }
    //      SmartDashboard.putNumber("pose distance", distance);

    //     if (distance > 4 && DriverStation.isEnabled()) {
    //         return false;
    //     }
    //     return LimelightHelpers.validPoseEstimate(estimate);

    // }
 
    @Logged
    public Pose2d getPose2d() {
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