package frc.robot.autos;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Physics {

    private final InterpolatingDoubleTreeMap velocitySpeed = new InterpolatingDoubleTreeMap();

    // Distance limits
    private static final double MIN_DISTANCE = 2.0;
    private static final double MAX_DISTANCE = 5.0;

    public Physics() {

        velocitySpeed.put(2.0, 38.0);
        velocitySpeed.put(2.25, 39.0);
        velocitySpeed.put(2.5, 40.0);
        velocitySpeed.put(2.75, 41.0);
        velocitySpeed.put(3.0, 42.0);
        velocitySpeed.put(3.25, 42.5);
        
        velocitySpeed.put(3.5, 43.0);
        velocitySpeed.put(3.75, 44.5);
        velocitySpeed.put(4.0, 46.0);
        velocitySpeed.put(4.25, 47.0);
        velocitySpeed.put(4.5, 48.0);
        velocitySpeed.put(4.75, 50.0);
        velocitySpeed.put(5.0, 52.0);
    }

    public double setVelocity(double distance) {

    // Clamp distance
    distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));

    // Get base interpolated velocity
    Double baseVelocity = velocitySpeed.get(distance);
    
    double offset = SmartDashboard.getNumber("Shooter Offset", 0.0);

    // Final velocity
    double finalVelocity = baseVelocity + offset;

    // Dashboard debugging
    SmartDashboard.putNumber("Shoot Distance", distance);
    SmartDashboard.putNumber("Base Velocity", baseVelocity);
    SmartDashboard.putNumber("Final Velocity", finalVelocity);

    return finalVelocity;
}
}