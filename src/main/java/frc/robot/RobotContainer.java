// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.simulation.PS5ControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.AutoShoot;
// import frc.robot.autos.AutoShoot;
// import frc.robot.autos.AlignToHubPoint;
import frc.robot.autos.Physics;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Hood;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
// import frc.robot.autos.AlignToHubPoint;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

@Logged
public class RobotContainer {

  private final CANBus rioCanBus = new CANBus("", "./logs/example.hoot");
  private final CANBus CarnivoreCanBus = new CANBus("Carnivore", "./logs/example.hoot");

  // Subsystem Imports
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Shooter shooter = new Shooter(rioCanBus);
  private final Kicker kicker = new Kicker(CarnivoreCanBus);
  private final Intake intake = new Intake(CarnivoreCanBus);
  private final IntakePivot intakepivot = new IntakePivot(rioCanBus);
  private final Indexer indexer = new Indexer(rioCanBus);
  private final Hood hood = new Hood(drivetrain, rioCanBus);

  // Swerve
  private double MaxSpeed =
      1
          * TunerConstants.kSpeedAt12Volts.in(
              MetersPerSecond); // kSpeedAt12Volts desired top speed //.6
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric robotRelativeDrive =
      new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // private final PS5Controller joystick = new PS5Controller(0);
  private final CommandPS5Controller joystick = new CommandPS5Controller(0);
  // private final CommandXboxController joystick = new CommandXboxController(1);
  private final CommandXboxController joystick2 = new CommandXboxController(1);

  // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Pigeon2 pigeon = drivetrain.getPigeon2();

  public final VisionPros visionpros = new VisionPros(drivetrain, pigeon);

  private final Physics physics = new Physics();

  // private final AutoShoot autoshoot = new AutoShoot(drivetrain, shooter, physics, null, null);

  private final Translation2d RED_HUB = new Translation2d(11.8, 4.025);
  private final Translation2d BLUE_HUB = new Translation2d(4.7, 4.025);

  //     private final Translation2d HUB_POSITION = new Translation2d(11.5, 4);
  //     private final Translation2d[] HUB_ALIGNMENT_POINTS = new Translation2d[] {
  //     //LeftTrench
  //     new Translation2d(12.7, 7.3),
  //     //RightTrench
  //     new Translation2d(12.7, 0.6),
  //     //Tower
  //     new Translation2d(14.7, 4.0),
  //     //Depot
  //     new Translation2d(15.7, 1),
  //     //Outpost
  //     new Translation2d(15.5, 7.3)

  // };

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    configureBindings();
    // hoodSafety(drivetrain, hood).schedule();
    // LimelightHelpers.SetRobotOrientation("limelight-left",drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    // Auto Selection is already handled by Glass.
    NamedCommands.registerCommand(
        "dumpMag", dumpMag(shooter, indexer, kicker, intakepivot, intake));
    NamedCommands.registerCommand("autoIntakeFuel", autoIntakeFuel(intake, intakepivot));
    NamedCommands.registerCommand("stopIntake", stopIntake(intake, intakepivot));
    // NamedCommands.registerCommand("intakeShuffle", intakeShuffle(intakepivot));
    // NamedCommands.registerCommand("aimAtTargetAuto",aimAtTargetAuto());
    NamedCommands.registerCommand(
        "print", Commands.runOnce(() -> System.out.println("commandsent")));
    autoChooser = AutoBuilder.buildAutoChooser();
    // See if you need this if the options for the autos are not popping up.
    autoChooser.setDefaultOption("1", Commands.print("1"));
    //  autoChooser.addOption("OutpostToMiddleOfHub", getAutonomousCommand());
    autoChooser.addOption("Dumpmagtest67", dumpMag(shooter, indexer, kicker, intakepivot, intake));
    autoChooser.addOption("autoIntakeFuel", autoIntakeFuel(intake, intakepivot));
    autoChooser.addOption("stopIntake", stopIntake(intake, intakepivot));
    // autoChooser.addOption("intakeShuffle", intakeShuffle(intakepivot));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));
    // shooter.setDefaultCommand(
    // new AutoShoot(drivetrain, shooter, physics, RED_HUB, BLUE_HUB)
    // );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on start.
    joystick.touchpad().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // REMOVED antijam haran
    // joystick.R3().whileTrue(indexer.setVelocity(-40).alongWith(kicker.setVelocity(-50).alongWith(shooter.setVelocity(-35))));

    // DRIVER BUTTONS aka Haran ==============================================
    // Left Bumper = AimAtHub
    joystick
        .L1()
        .whileTrue(
            aimAtHubMegaTag2(
                drivetrain, new Translation2d(11.8, 4.025), new Translation2d(4.7, 4.025)));
    // Left Trigger = Intake
    joystick.L2().whileTrue(intake.setVoltage(-8));
    // Right Trigger = Indexer + Kicker
    joystick
        .R2()
        .whileTrue(
            indexer
                .setVelocity(80)
                .alongWith(kicker.setVelocity(50))); // indexer velocity 45, kicker velocity 25
    // ,(intake.setVoltage(10))));
    // Right Bumper = Aim at Hub
    // joystick.R1()
    //     .whileTrue(aimAtHubMegaTag2(drivetrain, new Translation2d(11.8, 4.025), new
    // Translation2d(4.75,4.025)));
    joystick.R1().whileTrue(aimAtHubMegaTag2(drivetrain, RED_HUB, BLUE_HUB));
    // joystick.R1()
    //     .onTrue(new AutoShoot(drivetrain, shooter, physics, RED_HUB, BLUE_HUB));
    // .whileTrue(alignToClosestHubPoint());/
    // Face Buttons
    // joystick
    //     .triangle() //trench shot
    //     .onTrue(shooter.setVelocity(43));
    joystick.triangle().onTrue(new AutoShoot(drivetrain, shooter, physics, RED_HUB, BLUE_HUB));

    joystick
        .circle() // turn shooter off shot
        .onTrue(shooter.setVelocity(0));
    joystick.cross().onTrue(intakepivot.setPosition(26));
    joystick.square().onTrue(intakepivot.setPosition(0));
    // joystick
    //     .povDown()
    //     .onTrue(intake.setVoltage(6));
    // Driver DPAD buttons
    joystick.povUp().onTrue(intakeAgitate(intake, intakepivot));

    // Driver stick buttons
    joystick // slow button
        .L3()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX((-joystick.getLeftY() * MaxSpeed) * (.45))
                        .withVelocityY((-joystick.getLeftX() * MaxSpeed) * (.45))
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));
    joystick // fast button removed, unneeded
        .R3()
        .whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick2
    //     .povRight()
    //     .whileTrue(aimAtTargetMega(drivetrain, new Translation2d(12, 4)));
    // Driver Rumble

    // LUIS BUTTONS ===========================================
    joystick2.leftBumper().onTrue(shooter.setVelocity(43)); // left trench
    joystick2
        .rightBumper()
        .whileTrue(
            kicker.setVelocity(
                -50) /* .alongWith(shooter.setVelocity(-35))*/); // shooter & kicker anti
    // jam/reverse
    joystick2.rightTrigger().whileTrue(indexer.setVelocity(-40)); // spindexer anti jam/reverse
    joystick2.y().onTrue(shooter.setVelocity(40)); // tower shot
    joystick2.x().onTrue(shooter.setVelocity(46)); // depot shot
    joystick2.b().onTrue(shooter.setVelocity(44)); // outpost
    joystick2.a().onTrue(shooter.setVelocity(0)); // stops shooter
    // joystick2 //might need to remove
    //     .leftTrigger()
    //     .whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick2
    //     .povLeft()
    //     .onTrue(new AutoShoot(drivetrain, shooter, physics, RED_HUB, BLUE_HUB));

    // joystick2
    //     .rightTrigger()
    //     .whileTrue(indexer.setVelocity(45)
    //     .alongWith(kicker.setVelocity(25)));

    // joystick2.povUp()
    //     .onTrue(run shooter.increaseVelocityTest(100)));

    // joystick2.povDown()
    //     .onTrue(Commands.runOnce(() -> (-100)));

    // special buttons (start, select)

    drivetrain.registerTelemetry(logger::telemeterize);

    // Glass/SmartDashboard Buttons
    // The format is "(subsystem name) set (variable) (amount)""
    // Kicker buttons

    // SmartDashboard.putData("kicker velocity 5", kicker.setVelocity(-5));
    // SmartDashboard.putData("kicker velocity 20", kicker.setVelocity(-20));
    // SmartDashboard.putData("kicker velocity 45", kicker.setVelocity(-45));
    // SmartDashboard.putData("kicker velocity 50", kicker.setVelocity(-50));
    // SmartDashboard.putData("kicker velocity 60", kicker.setVelocity(-60));

    // SmartDashboard.putData("kicker set voltage 0V", kicker.setVoltage(0));
    // SmartDashboard.putData("kicker set voltage 1V", kicker.setVoltage(-1));
    // SmartDashboard.putData("kicker set voltage 3V", kicker.setVoltage(-3));
    // SmartDashboard.putData("kicker set voltage 4V", kicker.setVoltage(-4));
    // SmartDashboard.putData("kicker set voltage 6V", kicker.setVoltage(-6));
    // SmartDashboard.putData("kicker set voltage 9V", kicker.setVoltage(-9));
    // SmartDashboard.putData("kicker set voltage 12V", kicker.setVoltage(-12));

    // Shooter buttons
    SmartDashboard.putData("shooter velocity 5", shooter.setVelocity(5));
    SmartDashboard.putData("shooter velocity 20", shooter.setVelocity(20));
    SmartDashboard.putData("shooter velocity 25", shooter.setVelocity(25));
    SmartDashboard.putData("shooter velocity 30", shooter.setVelocity(30));
    SmartDashboard.putData("shooter velocity 31", shooter.setVelocity(31));
    SmartDashboard.putData("shooter velocity 32", shooter.setVelocity(32));
    SmartDashboard.putData("shooter velocity 33", shooter.setVelocity(33));
    SmartDashboard.putData("shooter velocity 34", shooter.setVelocity(34));
    SmartDashboard.putData("shooter velocity 35", shooter.setVelocity(35));
    SmartDashboard.putData("shooter velocity 36", shooter.setVelocity(36));
    SmartDashboard.putData("shooter velocity 37", shooter.setVelocity(37));
    SmartDashboard.putData("shooter velocity 38", shooter.setVelocity(38));
    SmartDashboard.putData("shooter velocity 39", shooter.setVelocity(39));
    SmartDashboard.putData("shooter velocity 40", shooter.setVelocity(40));
    SmartDashboard.putData("shooter velocity 41", shooter.setVelocity(41));
    SmartDashboard.putData("shooter velocity 42", shooter.setVelocity(42));
    SmartDashboard.putData("shooter velocity 43", shooter.setVelocity(43));
    SmartDashboard.putData("shooter velocity 44", shooter.setVelocity(44));
    SmartDashboard.putData("shooter velocity 45", shooter.setVelocity(45));
    SmartDashboard.putData("shooter velocity 46", shooter.setVelocity(46));
    SmartDashboard.putData("shooter velocity 47", shooter.setVelocity(47));
    SmartDashboard.putData("shooter velocity 48", shooter.setVelocity(48));
    SmartDashboard.putData("shooter velocity 49", shooter.setVelocity(49));
    SmartDashboard.putData("shooter velocity 50", shooter.setVelocity(50));
    SmartDashboard.putData("shooter velocity 51", shooter.setVelocity(51));
    SmartDashboard.putData("shooter velocity 52", shooter.setVelocity(52));
    SmartDashboard.putData("shooter velocity 53", shooter.setVelocity(53));
    SmartDashboard.putData("shooter velocity 54", shooter.setVelocity(54));
    SmartDashboard.putData("shooter velocity 55", shooter.setVelocity(55));

    // SmartDashboard.putData("shooter set voltage 0V", shooter.setVoltage(0));
    // SmartDashboard.putData("shooter set voltage .25V", shooter.setVoltage(0.25));
    // SmartDashboard.putData("shooter set voltage .50V", shooter.setVoltage(0.50));
    // SmartDashboard.putData("shooter set voltage .75V", shooter.setVoltage(0.75));
    // SmartDashboard.putData("shooter set voltage 1V", shooter.setVoltage(1));

    // SmartDashboard.putData("shooter set voltage 3V", shooter.setVoltage(3));
    // SmartDashboard.putData("shooter set voltage 4.5V", shooter.setVoltage(4.5));
    // SmartDashboard.putData("shooter set voltage 6V", shooter.setVoltage(6));
    // SmartDashboard.putData("shooter set voltage 9V", shooter.setVoltage(9));
    SmartDashboard.putData("shooter set voltage 12V", shooter.setVoltage(12));

    /*
    SmartDashboard.putData("set velocity 25 rps shooter", shooter.setVelocity(25));
    SmartDashboard.putData("set velocity 50 rps", shooter.setVelocity(50));
    SmartDashboard.putData("trench shot, set velocity 51 rps", shooter.setVelocity(51)); //Trench shot estimate
    SmartDashboard.putData("set velocity 52 rps", shooter.setVelocity(52));
     */

    // Intake Buttons
    SmartDashboard.putData("intake pivot reset encoder", intakepivot.resetEncoder());
    SmartDashboard.putData("intake pivot position 0", intakepivot.setPosition(0));
    SmartDashboard.putData("intake pivot position 8", intakepivot.setPosition(8));
    // SmartDashboard.putData("intake set voltage 0V", intake.setVoltage(0));
    // SmartDashboard.putData("intake set voltage 3V", intake.setVoltage(3));
    // SmartDashboard.putData("intake set voltage 6V", intake.setVoltage(6));
    // SmartDashboard.putData("intake set voltage 9V", intake.setVoltage(9));
    // SmartDashboard.putData("intake set voltage 12V", intake.setVoltage(12));

    // and so on so forth..
    // spindexer buttons
    // SmartDashboard.putData("spindexer velocity 20", indexer.setVelocity(20));
    // SmartDashboard.putData("spindexer velocity 40", indexer.setVelocity(40));
    // SmartDashboard.putData("spindexer velocity 45", indexer.setVelocity(45));
    // SmartDashboard.putData("spindexer velocity 50", indexer.setVelocity(50));

    // SmartDashboard.putData("spindexer set voltage 0V", indexer.setVoltage(0));
    // SmartDashboard.putData("spindexer set voltage 3V", indexer.setVoltage(3));
    // SmartDashboard.putData("spindexer set voltage 6V", indexer.setVoltage(6));
    // SmartDashboard.putData("spindexer set voltage 9V", indexer.setVoltage(9));
    // SmartDashboard.putData("spindexer set voltage 12V", indexer.setVoltage(12));
    // Hood buttons
    // SmartDashboard.putData("hood reset encoder", hood.resetEncoder());
    // SmartDashboard.putData("hood position 0", hood.setPosition(0));
    // SmartDashboard.putData("hood position 1", hood.setPosition(1));
    // SmartDashboard.putData("hood position 1.74, far trench shot", hood.setPosition(1.746));
    // SmartDashboard.putData("hood position 5", hood.setPosition(5));
    // SmartDashboard.putData("hood position 7", hood.setPosition(7));
    // SmartDashboard.putData("hood position 8", hood.setPosition(8));
    // SmartDashboard.putData("hood position 9", hood.setPosition(9));
    // SmartDashboard.putData("hood position 10", hood.setPosition(10));
    // gyro
    // SmartDashboard.getData("getRotation3d():" + pigeon.getRotation3d().getAngle());

    // SmartDashboard.putData("set hood from vision",
    // hood.setPosition(visionpros::getHoodAngleFromTy));

    // Named Commands, Necessary for autos!   NOT THE COMMANDS THEMSELVES, basically translating
    // them to be used on pathplanner

    // NamedCommands.registerCommand("Dump Mag", dumpMag(shooter, indexer, kicker));

  }

  // Auto
  public void autoInit() {
    drivetrain.seedFieldCentric(Rotation2d.fromDegrees(270)); // was 180
  }

  // Subsystem Commands
  // public Command loadFuel(){
  //     return kicker.setVoltage(3)
  //     .alongWith(
  //         indexer.setVoltage(3)
  //     );
  // }

  public Command dumpMag(
      Shooter shooter, Indexer indexer, Kicker kicker, IntakePivot intakepivot, Intake intake) {
    // return shooter.setVelocity(44)
    //     .alongWith(new
    // WaitCommand(1).andThen(indexer.setVelocity(45).alongWith(kicker.setVelocity(25))));
    return shooter
        .setVelocity(42)
        .alongWith(
            new WaitCommand(0.5) // reduced to 0.5 for reasons
                .andThen(kicker.setVelocity(25).alongWith(indexer.setVelocity(45))),
            new WaitCommand(6.0)
                .andThen(intakepivot.setPosition(0).alongWith(intake.setVoltageRun(0))))
        .withTimeout(7)
        .andThen(
            indexer
                .setVoltageRun(0)
                .alongWith(shooter.setVoltageRun(0), kicker.setVoltageRun(0))
                .withTimeout(0.1));
  }

  // public Command autoIntakeFuel(){
  //     return intake.setVoltage(12)
  //     .andThen(new WaitCommand(2))
  //     .andThen(intake.setVoltage(0));
  // }

  // rumble command should be tested on another command
  public Command rumbleCommand() {
    return new StartEndCommand(
            () -> joystick.setRumble(RumbleType.kBothRumble, 1.0),
            () -> joystick.setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(.8);
  }

  public Command autoIntakeFuel(Intake intake, IntakePivot intakepivot) {
    return intake.setVoltage(-7).alongWith(intakepivot.setPosition(26)).withTimeout(4.5);

    // .withTimeout(4)
    // .andThen(intakepivot.setPosition(0).alongWith(intake.setVoltageRun(0))); //test if
    // setVoltageRun is needed instead.
  }

  public Command stopIntake(Intake intake, IntakePivot intakepivot) {
    return intake.setVoltage(0).alongWith(intakepivot.setPosition(0)).withTimeout(3);
  }

  public Command intakeAgitate(Intake intake, IntakePivot intakepivot) {
    return intakepivot
        .setPosition(26)
        .until(() -> intakepivot.intakeAtPosition(26))
        .andThen(intakepivot.setPosition(13))
        .until(() -> intakepivot.intakeAtPosition(13))
        .andThen(intakepivot.setPosition(26))
        .until(() -> intakepivot.intakeAtPosition(26));
  }

  // public Command intakeShuffle(IntakePivot intakePivot) {
  //     return intakepivot.setPosition(0)
  //         .alongWith(new WaitCommand(1)
  //             .andThen(intakepivot.setPosition(26)))
  //             .withTimeout(1)
  //             .andThen(intakepivot.setPosition(0))
  //             .withTimeout(1)
  //             .andThen(intakepivot.setPosition(26))
  //             .withTimeout(1)
  //             .andThen(intakepivot.setPosition(0))
  //             .withTimeout(1)
  //             .andThen(intakepivot.setPosition(26))
  //             .withTimeout(1)
  //             .andThen(intakepivot.setPosition(0));

  // }

  // public Command intakeFuel(){
  //     return intake.setVoltage(3)
  //     .alongWith(
  //     intakepivot.setPosition(0.7));
  // }

  /* code from kevlio pay no mind
  *  return funnel.runFunnel(0.4).alongWith(intake.runIntake(0.2))
         .until(intake::hasCoral)
         .andThen(new WaitCommand(0.10))
         .andThen(funnel.stopFunnel().alongWith(intake.stopIntake()));
  */

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

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

  // back up if LL aint working
  //   public Command aimAtTargetMT1(CommandSwerveDrivetrain drivetrain) {
  //     return drivetrain.applyRequest(
  //         () -> {
  //           double kP = .03; // kp was .0176
  //           double targetingAngularVelocity = LimelightHelpers.getTX("limelight-left") * kP;
  //           targetingAngularVelocity *= MaxAngularRate;
  //           targetingAngularVelocity *= -1.0;
  //           return drive
  //               .withVelocityX(
  //                   -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
  //               .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X
  // (left)
  //               .withRotationalRate(
  //                   targetingAngularVelocity); // Drive counterclockwise with negative X (left)
  //         });
  //   }

  // for auto
  public Command aimAtHubAutoMT2(
      CommandSwerveDrivetrain drivetrain, Translation2d redTarget, Translation2d blueTarget) {
    return drivetrain.applyRequest(
        () -> {
          var alliance = DriverStation.getAlliance();
          var target = redTarget;

          if (alliance.isPresent()) {

            if (alliance.get().equals(Alliance.Blue)) {
              target = blueTarget;
            }
          }

          var currentPose = drivetrain.getState().Pose;
          var direction = target.minus(currentPose.getTranslation());
          var error = direction.getAngle().minus(currentPose.getRotation()).getRadians();

          double kP = 3; // kp was .0176
          double targetingAngularVelocity = error * kP;
          // targetingAngularVelocity *= MaxAngularRate;
          // targetingAngularVelocity *= -1.0;

          // var angle = Math.atan2(, )

          return drive
              .withVelocityX(0) // Drive forward with negative Y (forward)
              .withVelocityY(0) // Drive left with negative X (left)
              .withRotationalRate(
                  targetingAngularVelocity); // Drive counterclockwise with negative X (left)
        });
  }

  // do not use
  //  public Command aimAtHubno(CommandSwerveDrivetrain drivetrain) { //what does this command even
  // do??
  //     return  drivetrain.applyRequest(
  //                 () -> {

  //                      var currentPose = drivetrain.getState().Pose;
  //                      var targetTranslation = new Translation2d(4.5, 4); //blue
  //                      var direction =
  //                         targetTranslation.minus(currentPose.getTranslation());
  //                     var error =
  //                         direction.getAngle()
  //                             .minus(currentPose.getRotation())
  //                             .getRadians();

  //                     double kP = .35; //kp was .0176
  //                     double targetingAngularVelocity = error * kP;
  //                     // targetingAngularVelocity *= MaxAngularRate;
  //                     // targetingAngularVelocity *= -1.0;

  //                     // var angle = Math.atan2(, )

  //                     return drive
  //                         .withVelocityX(
  //                             -joystick.getLeftY()
  //                                 * MaxSpeed) // Drive forward with negative Y (forward)
  //                         .withVelocityY(
  //                             -joystick.getLeftX() * MaxSpeed) // Drive left with negative X
  // (left)
  //                         .withRotationalRate(
  //                             targetingAngularVelocity); // Drive counterclockwise with negative
  // X (left)
  //                 });
  //             }
  public Command aimAtHubMegaTag2(
      CommandSwerveDrivetrain drivetrain, Translation2d redTarget, Translation2d blueTarget) {
    return drivetrain.applyRequest(
        () -> {
          var alliance = DriverStation.getAlliance();
          var target = redTarget;

          if (alliance.isPresent()) {

            if (alliance.get().equals(Alliance.Blue)) {
              target = blueTarget;
            }
          }

          var currentPose = drivetrain.getState().Pose;
          var direction = target.minus(currentPose.getTranslation());
          var error = direction.getAngle().minus(currentPose.getRotation()).getRadians();

          double kP = 3; // kp was .0176
          double targetingAngularVelocity = error * kP;
          // targetingAngularVelocity *= MaxAngularRate;
          // targetingAngularVelocity *= -1.0;

          // var angle = Math.atan2(, )

          return drive
              .withVelocityX(
                  -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(
                  targetingAngularVelocity); // Drive counterclockwise with negative X (left)
        });
  }

  // public Command driveToDistance(CommandSwerveDrivetrain drivetrain, Translation2d redTarget,
  // Translation2d blueTarget) {
  //         return  drivetrain.applyRequest(
  //                     () -> {

  //                         var KpDistance = -0.1f;  // Proportional control constant for distance
  //                         var current_distance = Estimate_Distance();  // see the 'Case Study:
  // Estimating Distance'

  //               if (joystick->GetRawButton(9)) {
  //                 float distance_error = desired_distance - current_distance;
  //             //   driving_adjust = KpDistance * distance_error;
  //             //   left_command += distance_adjust;
  //             //   right_command += distance_adjust;

  //                     });
  //                 }

  // public Command aimAtOutpostMegaTag2(CommandSwerveDrivetrain drivetrain, Translation2d target) {
  //     return  drivetrain.applyRequest(
  //                 () -> {

  //                      var currentPose = drivetrain.getState().Pose;
  //                      var direction =
  //                         target.minus(currentPose.getTranslation());
  //                     var error =
  //                         direction.getAngle()
  //                             .minus(currentPose.getRotation())
  //                             .getRadians();

  //                     double kP = 3; //kp was .0176
  //                     double targetingAngularVelocity = error * kP;
  //                     // targetingAngularVelocity *= MaxAngularRate;
  //                     // targetingAngularVelocity *= -1.0;

  //                     // var angle = Math.atan2(, )

  //                     return drive
  //                         .withVelocityX(
  //                             -joystick.getLeftY()
  //                                 * MaxSpeed) // Drive forward with negative Y (forward)
  //                         .withVelocityY(
  //                             -joystick.getLeftX() * MaxSpeed) // Drive left with negative X
  // (left)
  //                         .withRotationalRate(
  //                             targetingAngularVelocity); // Drive counterclockwise with negative
  // X (left)
  //                 });
  //             }

  //     private Command alignToClosestHubPoint() {
  //         return new AlignToHubPoint(
  //         drivetrain,
  //         HUB_ALIGNMENT_POINTS[0],   // dummy value
  //         HUB_ALIGNMENT_POINTS,
  //         HUB_POSITION
  //         );
  // }

}
