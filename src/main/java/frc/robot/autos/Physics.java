// package frc.robot.autos;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix6.hardware.Pigeon2;

// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Hood;
// import frc.robot.generated.TunerConstants;
// import frc.robot.LimelightHelpers;
// import frc.robot.VisionPros;


// import com.ctre.phoenix6.swerve.SwerveDrivetrain;
// import com.pathplanner.lib.path.PathPlannerPath;

// public class Physics {
//     private final Hood hood;
//     private final Shooter shooter;
//     private final CommandSwerveDrivetrain drivetrain;
//     private double extraDistance = 0;
//     private double velocity = 15;
//     private boolean autoAimEnabled = true;
//     public static boolean dynamicMode = false;
//     private double[] vXYZ = new double[3];

// public Physics(CommandSwerveDrivetrain drivetrain)  {
//     hood = new Hood(drivetrain);
//     shooter = new Shooter();
//     this.drivetrain = drivetrain;
// }
// public double getX() {
//         // double x = aimMode != AimMode.VISION ? positionTracker.getPosition()[1] :
//            double x = LimelightHelpers.getTX("limelight-left");
//            SmartDashboard.putNumber("x",x);
           
//             // estimate.pose.getTranslation().getDistance(currentPose.getTranslation());

//         //         getLLDistance() * Math.cos(Math.toRadians(getLLTurretAngle()));
//         // tab.setEntry("x", x);
//         return x + extraDistance;
//     }
// public double getY() {
//            double y = LimelightHelpers.getTY("limelight-left");
//             // estimate.pose.getTranslation().getDistance(currentPose.getTranslation());

//         //         getLLDistance() * Math.cos(Math.toRadians(getLLTurretAngle()));
//         // tab.setEntry("x", x);
//         return y;
//     }
// public double getZ() {
//            double Z = LimelightHelpers.getTX("limelight-left")* Math.tan(Math.toRadians(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()));
//             // estimate.pose.getTranslation().getDistance(currentPose.getTranslation());

//         //         getLLDistance() * Math.cos(Math.toRadians(getLLTurretAngle()));
//         // tab.setEntry("x", x);
//         return Z;
//     }
//  public double getDistanceToTarget() {
//     double yAngle = getY();
//     double limelightHeight = 0.3; // meters, adjust for your robot
//     double targetHeight = 2.64;   // meters, adjust for your target
//     double angleToTarget = Math.toRadians(yAngle + drivetrain.getPigeon2().getYaw().getValueAsDouble()); // total angle
//     double distance = (targetHeight - limelightHeight) / Math.tan(angleToTarget);

//     SmartDashboard.putNumber("Target Dist", distance);
//     return distance;
// }
//  public double getCalculatedHoodAngle() {
//         // Returns the hood angle using the relationship between horizontal and vertical velocities
//         double hoodAngle = Math.toDegrees(Math.atan2(vXYZ[2], Math.sqrt(vXYZ[0] * vXYZ[0] + vXYZ[1] * vXYZ[1])));
//         if (Math.abs(driveTrain.getYVelocity()) > 0.1) {
//             hoodAngle += (driveTrain.getYVelocity());
//         }
//         if (getX() - extraDistance > 2) {
//             hoodAngle += (getX() * getX()) * 0.2;
//         }
//         return hoodAngle + hoodAng;
//     }

// }

