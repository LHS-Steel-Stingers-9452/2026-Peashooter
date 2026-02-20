package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.Rotations;
// import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
// import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Pivot subsystem using TalonFX with Krakenx60 motor
 */
@Logged(name = "Shooter")
public class Shooter extends SubsystemBase {

  // Constants
  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);
  private final int canID = 17;
  private final int canID2 = 18;
  private final double gearRatio = 1;
  private final double kP = 0.2; // //70 started at 1, should improve recovery
  private final double kI = 0;
  private final double kD = 0; //0.75//helped with reducing noise, somehwat
  private final double kS = 0;
  private final double kV = 0.127; //voltage, divide voltage by velocity
  private final double kA = 0; //chat said try 1, removed for now bcuz extra variable
  // private final double kG = 0; // Unused for pivots
  // private final double maxVelocity = 1; // rad/s
  // private final double maxAcceleration = 1; // rad/s²
  private final boolean brakeMode = false;
  private final boolean enableStatorLimit = true;
  private final double statorCurrentLimit = 60;
  private final boolean enableSupplyLimit = false;
  private final double supplyCurrentLimit = 40;


  // Motor controller
  private final TalonFX motor;
  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  private final TalonFX motor2;
  private final StatusSignal<Angle> positionSignal2;
  private final StatusSignal<AngularVelocity> velocitySignal2;
  private final StatusSignal<Voltage> voltageSignal2;
  private final StatusSignal<Current> statorCurrentSignal2;
  private final StatusSignal<Temperature> temperatureSignal2;

  // Simulation
  private final SingleJointedArmSim pivotSim;

  /**
   * Creates a new Pivot Subsystem.
   */
  public Shooter() {
    // Initialize motor controller
    motor = new TalonFX(canID);
    motor2 = new TalonFX(canID2);

    // Create control requests
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    positionSignal2 = motor2.getPosition();
    velocitySignal2 = motor2.getVelocity();
    voltageSignal2 = motor2.getMotorVoltage();
    statorCurrentSignal2 = motor2.getStatorCurrent();
    temperatureSignal2 = motor2.getDeviceTemp();

    // Set automatic update rates (Hz)
    positionSignal.setUpdateFrequency(20);
    velocitySignal.setUpdateFrequency(20);
    voltageSignal.setUpdateFrequency(10);
    statorCurrentSignal.setUpdateFrequency(10);
    temperatureSignal.setUpdateFrequency(5);
    // Set automatic update rates (Hz)
    positionSignal2.setUpdateFrequency(20);
    velocitySignal2.setUpdateFrequency(20);
    voltageSignal2.setUpdateFrequency(10);
    statorCurrentSignal2.setUpdateFrequency(10);
    temperatureSignal2.setUpdateFrequency(5);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Configure PID for slot 0
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    // Set current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set brake mode
    config.MotorOutput.NeutralMode = brakeMode
      ? NeutralModeValue.Brake
      : NeutralModeValue.Coast;

    // Apply gear ratio
    config.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    motor.getConfigurator().apply(config);

    // Reset encoder position
    motor.setPosition(0);

    // Initialize simulation
    pivotSim = new SingleJointedArmSim(
      dcMotor, // Motor type
      gearRatio,
      0.01, // Arm moment of inertia - Small value since there are no arm parameters
      0.1, // Arm length (m) - Small value since there are no arm parameters
      Units.degreesToRadians(-90), // Min angle (rad)
      Units.degreesToRadians(90), // Max angle (rad)
      false, // Simulate gravity - Disable gravity for pivot
      Units.degreesToRadians(0) // Starting position (rad)
    );
      //Second motor became opposed
    motor2.setControl(new Follower(canID, MotorAlignmentValue.Opposed));
  }

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {
    
  }
 
  /**
   * Get the current position in Rotations.
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations
    return positionSignal.getValueAsDouble();
  }

  @Logged(name = "Position2/Rotations")
  public double getPosition2() {
    // Rotations
    return positionSignal2.getValueAsDouble();
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  @Logged(name = "Velocity2")
  public double getVelocity2() {
    return velocitySignal2.getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  @Logged(name = "Voltage2")
  public double getVoltage2() {
    return voltageSignal2.getValueAsDouble();
  }
  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  @Logged(name = "Current")
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  @Logged(name = "Current2")
  public double getCurrent2() {
    return statorCurrentSignal2.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  @Logged(name = "Temperature")
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  @Logged(name = "Temperature2")
  public double getTemperature2() {
    return temperatureSignal2.getValueAsDouble();
  }


  

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  /*public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
*/
  public Command setVoltage(double voltage) {
    return runOnce(() -> motor.setVoltage(voltage));
  }

  /**
   * Creates a command to stop the pivot.
   * @return A command that stops the pivot
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the pivot at a specific velocity.
   * @return A command that moves the pivot at the specified velocity
   */
  public Command setVelocity(double velocity) {
    return run(() ->  motor.setControl(velocityRequest.withVelocity(velocity)));
  }
}
