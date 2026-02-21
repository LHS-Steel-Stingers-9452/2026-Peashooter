// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import javax.crypto.spec.DHGenParameterSpec;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot; 

@Logged
public class RobotContainer {
//Subsystem Imports
    // private final Climber climber = new Climber();
    private final Shooter shooter = new Shooter();
    private final Kicker kicker = new Kicker();
    private final Intake intake = new Intake();
    private final IntakePivot intakepivot = new IntakePivot();
    private final Indexer indexer = new Indexer();
    private final Hood hood = new Hood();
    

//Swerve
    private double MaxSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed //.6
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
     private final SwerveRequest.RobotCentric robotRelativeDrive =
      new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final VisionPros visionpros = new VisionPros(drivetrain);

    
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        configureBindings();
        hoodSafety(drivetrain, hood).schedule();
        // LimelightHelpers.SetRobotOrientation("limelight-left",drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        //Auto Selection is already handled by Glass.
        autoChooser = AutoBuilder.buildAutoChooser();
        //See if you need this if the options for the autos are not popping up.
        //  autoChooser.setDefaultOption("1", Commands.print("1"));
        //  autoChooser.addOption("BlueTopAuto", getAutonomousCommand());
        // SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

   
//Joystick/controller buttons    
        
    //Left Bumper = AimAtOutpost
        joystick.leftBumper()
        .whileTrue(aimAtOutpost(drivetrain));
    //Left Trigger = Intake
        joystick
            .leftTrigger()
            .whileTrue(intake.setVoltage(10));
    //Right Trigger = Indexer + Kicker  
        joystick
            .rightTrigger() 
            .whileTrue(indexer.setVelocity(45)
            .alongWith(kicker.setVelocity(60)));
            // ,(intake.setVoltage(10))));
    //Right Bumper = Aim at Hub
        joystick 
            .rightBumper()
            .whileTrue(aimAtTarget(drivetrain, new Translation2d(12, 4)));
    //Face Buttons
    //face buttons (a, b, x, y) y is outpost, b is trench, a is fender
        joystick
            .y()
            .onTrue(shooter.setVelocity(44));
        joystick
            .b()
            .onTrue(shooter.setVelocity(0));
        // joystick        
        //     .b()
        //     .whileTrue(
        //      drivetrain.applyRequest(
        //         () -> 
        //          drive
        //         .withVelocityX(
        //            (-joystick.getLeftY() * MaxSpeed)*(.45)) 
        //         .withVelocityY(
        //             (-joystick.getLeftX() * MaxSpeed)*(.45))
        //         .withRotationalRate(
        //             -joystick.getRightX()
        //                 *MaxAngularRate)
        //     ));
        joystick
            .a()
            .onTrue(intakepivot.setPosition(8));
        joystick
            .x()
            .onTrue(intakepivot.setPosition(0));
        // joystick
        //     .x()
        //     .onTrue(getAutonomousCommand()); //Anti jam thing
    



        // joystick  
        //     .a()
        //     .onTrue(shooter.setVelocity(0));   //may not be physically possible
        // joystick
        //     .b()
        //     .onTrue(shooter.setVelocity(39));
        // // joystick
        // //     .x()
        // //     .whileTrue(getAutonomousCommand());
        // joystick
        //     .y()
        //     .onTrue(shooter.setVelocity(44));


            // joystick
            //     .povUp()
            //     .onTrue(getAutonomousCommand());
            
    //D Pad Buttons
    //     joystick
    //         .povUp()
    //         .whileTrue(shooter.setVoltage(0));
    //         //.whileTrue(climber.setVoltage(0.67));
        // joystick
        //     .povRight()
        //     .whileTrue(intakepivot.setPosition(-0.67).alongWith(hood.setPosition(-0.67)));
        // joystick
        //     .povDown()
        //     .whileTrue(climber.setVoltage(-0.67));
        // joystick
        //     .povLeft()
        //     .whileTrue(drivetrain.applyRequest(() -> brake));

            
    //special buttons (start, select)    
    
        drivetrain.registerTelemetry(logger::telemeterize);

    //Glass/SmartDashboard Buttons
                    // The format is "(subsystem name) set (variable) (amount)""
        //Kicker buttons
            SmartDashboard.putData("kicker velocity 5", kicker.setVelocity(-5));
            SmartDashboard.putData("kicker velocity 20", kicker.setVelocity(-20));
            SmartDashboard.putData("kicker velocity 45", kicker.setVelocity(-45));
            SmartDashboard.putData("kicker velocity 50", kicker.setVelocity(-50));
            SmartDashboard.putData("kicker velocity 60", kicker.setVelocity(-60));
            
                /* 
                    SmartDashboard.putData("kicker set voltage 0V", kicker.setVoltage(0));
                    SmartDashboard.putData("kicker set voltage 1V", kicker.setVoltage(-1));
                    SmartDashboard.putData("kicker set voltage 3V", kicker.setVoltage(-3));
                    SmartDashboard.putData("kicker set voltage 4V", kicker.setVoltage(-4));
                    SmartDashboard.putData("kicker set voltage 6V", kicker.setVoltage(-6));
                    SmartDashboard.putData("kicker set voltage 9V", kicker.setVoltage(-9));
                    SmartDashboard.putData("kicker set voltage 12V", kicker.setVoltage(-12));
                    */
            
        //Shooter buttons
            SmartDashboard.putData("shooter velocity 5", shooter.setVelocity(5));
            SmartDashboard.putData("shooter velocity 20", shooter.setVelocity(20));
            SmartDashboard.putData("shooter velocity 25", shooter.setVelocity(25));
            SmartDashboard.putData("shooter velocity 30", shooter.setVelocity(30));
            SmartDashboard.putData("shooter velocity 40", shooter.setVelocity(40));
            SmartDashboard.putData("shooter velocity 41", shooter.setVelocity(41));
            SmartDashboard.putData("shooter velocity 42", shooter.setVelocity(42));
            SmartDashboard.putData("shooter velocity 43", shooter.setVelocity(43));
            SmartDashboard.putData("shooter velocity 44", shooter.setVelocity(44));
            SmartDashboard.putData("shooter velocity 45", shooter.setVelocity(45));
            SmartDashboard.putData("shooter velocity 50", shooter.setVelocity(50));

            SmartDashboard.putData("shooter set voltage 0V", shooter.setVoltage(0));
            SmartDashboard.putData("shooter set voltage .25V", shooter.setVoltage(0.25));
            SmartDashboard.putData("shooter set voltage .50V", shooter.setVoltage(0.50));
            SmartDashboard.putData("shooter set voltage .75V", shooter.setVoltage(0.75));
            SmartDashboard.putData("shooter set voltage 1V", shooter.setVoltage(1));
            
            SmartDashboard.putData("shooter set voltage 3V", shooter.setVoltage(3));
            SmartDashboard.putData("shooter set voltage 4.5V", shooter.setVoltage(4.5));
            SmartDashboard.putData("shooter set voltage 6V", shooter.setVoltage(6));
            SmartDashboard.putData("shooter set voltage 9V", shooter.setVoltage(9));
            SmartDashboard.putData("shooter set voltage 12V", shooter.setVoltage(12));
            
             
           
            /*
            SmartDashboard.putData("set velocity 25 rps shooter", shooter.setVelocity(25));
            SmartDashboard.putData("set velocity 50 rps", shooter.setVelocity(50));
            SmartDashboard.putData("trench shot, set velocity 51 rps", shooter.setVelocity(51)); //Trench shot estimate
            SmartDashboard.putData("set velocity 52 rps", shooter.setVelocity(52));
             */

        //Intake Buttons
            SmartDashboard.putData("intake pivot reset encoder", intakepivot.resetEncoder());
            SmartDashboard.putData("intake pivot position 0", intakepivot.setPosition(0));
            SmartDashboard.putData("intake pivot position 8", intakepivot.setPosition(8));
            SmartDashboard.putData("intake set voltage 0V", intake.setVoltage(0));
            SmartDashboard.putData("intake set voltage 3V", intake.setVoltage(3));
            SmartDashboard.putData("intake set voltage 6V", intake.setVoltage(6));
            SmartDashboard.putData("intake set voltage 9V", intake.setVoltage(9));
            SmartDashboard.putData("intake set voltage 12V", intake.setVoltage(12));

             //and so on so forth..
        //spindexer buttons
            SmartDashboard.putData("spindexer velocity 20", indexer.setVelocity(20));
            SmartDashboard.putData("spindexer velocity 40", indexer.setVelocity(40));
            SmartDashboard.putData("spindexer velocity 45", indexer.setVelocity(45));
            SmartDashboard.putData("spindexer velocity 50", indexer.setVelocity(50));

            SmartDashboard.putData("spindexer set voltage 0V", indexer.setVoltage(0));
            SmartDashboard.putData("spindexer set voltage 3V", indexer.setVoltage(3));
            SmartDashboard.putData("spindexer set voltage 6V", indexer.setVoltage(6));
            SmartDashboard.putData("spindexer set voltage 9V", indexer.setVoltage(9));
            SmartDashboard.putData("spindexer set voltage 12V", indexer.setVoltage(12));
        //Hood buttons
            SmartDashboard.putData("hood reset encoder", hood.resetEncoder());
            SmartDashboard.putData("hood position 1", hood.setPosition(1));
            SmartDashboard.putData("hood position 5", hood.setPosition(5));
            SmartDashboard.putData("hood position 7", hood.setPosition(7));
            SmartDashboard.putData("hood position 8", hood.setPosition(8));
            SmartDashboard.putData("hood position 9", hood.setPosition(9));
            SmartDashboard.putData("hood position 10", hood.setPosition(10));






    }
    //Auto
     public void autoInit(){
                    drivetrain.seedFieldCentric();
    }





    //Subsystem Commands
    public Command loadFuel(){
        return kicker.setVoltage(3)
        .alongWith(
            indexer.setVoltage(3) 
        );
    }

    public Command shootFuel(){
        return shooter.setVelocity(1.67); 
    }

    public Command intakeFuel(){
        return intake.setVoltage(3)
        .alongWith(
        intakepivot.setPosition(0.7));
    } 


                /*
             *  return funnel.runFunnel(0.4).alongWith(intake.runIntake(0.2))
                    .until(intake::hasCoral)
                    .andThen(new WaitCommand(0.10))
                    .andThen(funnel.stopFunnel().alongWith(intake.stopIntake()));
             */

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
    }

    public Command aimAtTarget(CommandSwerveDrivetrain drivetrain) {
        return  drivetrain.applyRequest(
                    () -> {
                        double kP = .03; //kp was .0176
                        double targetingAngularVelocity = LimelightHelpers.getTX("limelight-left") * kP;
                        targetingAngularVelocity *= MaxAngularRate;
                        targetingAngularVelocity *= -1.0;
                        return drive
                            .withVelocityX(
                                -joystick.getLeftY()
                                    * MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(
                                -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(
                                targetingAngularVelocity); // Drive counterclockwise with negative X (left)
                    });
    }

     public Command aimAtHub(CommandSwerveDrivetrain drivetrain) {
        return  drivetrain.applyRequest(
                    () -> {
                        
                         var currentPose = drivetrain.getState().Pose;
                         var targetTranslation = new Translation2d(13, 4); // We got (12, 4) IRL but (13, 4) works better in sim, we can play with it
                         var direction =
                            targetTranslation.minus(currentPose.getTranslation());
                        var error =
                            direction.getAngle()
                                .minus(currentPose.getRotation())
                                .getRadians();

                        double kP = .35; //kp was .0176
                        double targetingAngularVelocity = error * kP;
                        // targetingAngularVelocity *= MaxAngularRate;
                        // targetingAngularVelocity *= -1.0;

                        // var angle = Math.atan2(, )

                        
                        return drive
                            .withVelocityX(
                                -joystick.getLeftY()
                                    * MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(
                                -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(
                                targetingAngularVelocity); // Drive counterclockwise with negative X (left)
                    });
                }
    public Command aimAtTarget(CommandSwerveDrivetrain drivetrain, Translation2d target) {
        return  drivetrain.applyRequest(
                    () -> {
                        
                         var currentPose = drivetrain.getState().Pose;
                         var direction =
                            target.minus(currentPose.getTranslation());
                        var error =
                            direction.getAngle()
                                .minus(currentPose.getRotation())
                                .getRadians();
                                
                        double kP = .35; //kp was .0176
                        double targetingAngularVelocity = error * kP;
                        // targetingAngularVelocity *= MaxAngularRate;
                        // targetingAngularVelocity *= -1.0;

                        // var angle = Math.atan2(, )

                        
                        return drive
                            .withVelocityX(
                                -joystick.getLeftY()
                                    * MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(
                                -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(
                                targetingAngularVelocity); // Drive counterclockwise with negative X (left)
                    });
                }
    public Command aimAtOutpost(CommandSwerveDrivetrain drivetrain) {
        return  drivetrain.applyRequest(
                    () -> {
                        

                        var currentPose = drivetrain.getState().Pose;
                        var currentAngle = currentPose.getRotation().getRadians();
                        var poseX = currentPose.getX();
                        var poseY = currentPose.getY();

                        var targetX = 12; //get cords
                        var targetY = 4; //get cords

                        var angle = Math.atan2(poseX - targetX, poseY - targetY);
                        var error = currentAngle - angle;
                        
                        SmartDashboard.putNumber("angle", angle);
                        SmartDashboard.putNumber("current angle", currentAngle);
                        SmartDashboard.putNumber("error", error);

                       

                        double kP = .35; //kp was .0176
                        double targetingAngularVelocity = error * kP;
                        // targetingAngularVelocity *= MaxAngularRate;
                        // targetingAngularVelocity *= -1.0;

                        // var angle = Math.atan2(, )

                        
                        return drive
                            .withVelocityX(
                                -joystick.getLeftY()
                                    * MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(
                                -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(
                                targetingAngularVelocity); // Drive counterclockwise with negative X (left)
                    });
                }
    public Command hoodSafety(CommandSwerveDrivetrain drivetrain, Hood hood) {
            return Commands.run(() -> {

                var currentPose = drivetrain.getState().Pose;
                double poseX = currentPose.getX();
                double poseY = currentPose.getY();

                double radius = 4;

                double target1X = 12; //real cords for both trenches, target 1 is left, target 2 is right.
                double target1Y = 0.6;
                double target2X = 12;
                double target2Y = 7.2;

                // Distance first zone
                double dx1 = poseX - target1X;
                double dy1 = poseY - target1Y;
                double distance1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);

                // Distance second zone
                double dx2 = poseX - target2X;
                double dy2 = poseY - target2Y;
                double distance2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);

                if (distance1 <= radius || distance2 <= radius) {
                    hood.setPosition(1);  // DOWN
                } else {
                    hood.setPosition(10);  // UP
                }

        }, hood);
        }
}
