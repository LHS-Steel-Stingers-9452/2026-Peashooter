// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
//Subsystem Imports
    private final Shooter shooter = new Shooter();
    private final Kicker kicker = new Kicker();
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

    public RobotContainer() {
        configureBindings();
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

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // vision bindings, for driver( he doesn't even use it T-T )

    //lock onto april tag
         joystick
            .rightTrigger()
            .whileTrue(aimAtTarget(drivetrain));
               
                    

        drivetrain.registerTelemetry(logger::telemeterize);
    //Glass/SmartDashboard Buttons
            // SmartDashboard.putData("set voltage 0V", shooter.setVoltage(0));
        //Kicker buttons
            SmartDashboard.putData("set voltage 1V kicker", kicker.setVoltage(-1));
            SmartDashboard.putData("set voltage 3V kicker", kicker.setVoltage(-3));
            SmartDashboard.putData("set voltage 6V kicker", kicker.setVoltage(-6));
            SmartDashboard.putData("set voltage 9V kicker", kicker.setVoltage(-9));
            SmartDashboard.putData("set voltage 12V kicker", kicker.setVoltage(-12));
            // SmartDashboard.putData("set velocity 5", shooter.moveAtVelocityCommand(5));
            // SmartDashboard.putData("set velocity 4.6", shooter.moveAtVelocityCommand(4.6));
        //Shooter buttons
            SmartDashboard.putData("set velocity 25 rps shooter", shooter.moveAtVelocityCommand(25));
            // SmartDashboard.putData("set velocity 50 rps", shooter.moveAtVelocityCommand(50));
            SmartDashboard.putData("trench shot, set velocity 51 rps", shooter.moveAtVelocityCommand(51)); //Trench shot estimate
            
            SmartDashboard.putData("set velocity 52 rps", shooter.moveAtVelocityCommand(52));
            SmartDashboard.putData("set velocity 53 rps", shooter.moveAtVelocityCommand(53));
            SmartDashboard.putData("set velocity 54 rps", shooter.moveAtVelocityCommand(54));
            SmartDashboard.putData("set velocity 55 rps", shooter.moveAtVelocityCommand(55));
            SmartDashboard.putData("set velocity 57 rps", shooter.moveAtVelocityCommand(57));
            SmartDashboard.putData("set velocity 60 rps", shooter.moveAtVelocityCommand(60));
            SmartDashboard.putData("set velocity 65 rps", shooter.moveAtVelocityCommand(65));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
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
}
