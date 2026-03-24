package frc.robot.autos;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Logged

public class Physics {
    private final InterpolatingDoubleTreeMap velocitySpeed = new InterpolatingDoubleTreeMap();

    // Distance limits
    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 10;

    public Physics() {
        velocitySpeed.put(1.8, 33.0); //use
        // velocitySpeed.put(2.0, 35.2); //use
        velocitySpeed.put(2.1, 35.0);
        velocitySpeed.put(2.9, 39.0); 
        velocitySpeed.put(3.8, 43.0); 
        velocitySpeed.put(4.56, 45.0); 
        velocitySpeed.put(6.1, 47.0); //38.0
        velocitySpeed.put(6.59, 50.0);
        velocitySpeed.put(6.8, 55.0);
        // velocitySpeed.put(3.8, 42.0); //use
        // velocitySpeed.put(4.0, 42.4);
        // velocitySpeed.put(4.25, 43.3);
        // velocitySpeed.put(4.56, 45.0); //use
        // velocitySpeed.put(4.75, 45.0);
        // velocitySpeed.put(5.0, 45.9);
        // velocitySpeed.put(6.59, 50.0); //use
        // velocitySpeed.put(6.8, 55.0); //use
        // velocitySpeed.put(7.5, 57.0); //use



    }

    public double setVelocity(double distance) {

    // Clamp distance
    distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));

    // Get base interpolated velocity
    Double baseVelocity = velocitySpeed.get(distance);
    
    double offset = 3;

    // Final velocity
    double finalVelocity = baseVelocity + offset;

    // Dashboard debugging
    SmartDashboard.putNumber("Shoot Distance", distance);
    SmartDashboard.putNumber("Base Velocity", baseVelocity);
    SmartDashboard.putNumber("Final Velocity", finalVelocity);
    SmartDashboard.getNumber("Shoot Distance", distance);
    SmartDashboard.getNumber("Base Velocity", baseVelocity);
    SmartDashboard.getNumber("Final Velocity", finalVelocity);

    return finalVelocity;
}
}