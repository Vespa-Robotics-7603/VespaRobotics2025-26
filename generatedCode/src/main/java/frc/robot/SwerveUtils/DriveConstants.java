package frc.robot.SwerveUtils;

// import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveConstants {
    // K values
    public static final double KP = 0.25;
    public static final double KI = 0;
    public static final double KD = 15;
    public static final double KS = 0;
    public static final double KV = 0.124;

    // Maximum rate of acceleration per second (i.e., by how much should we
    // allow the requested X and Y values to change per second when speeding
    // up from a stop)
    // public static final double MAXACCELRATE = 0.5;
    // public static SlewRateLimiter ACCELLIMITER = new SlewRateLimiter(MAXACCELRATE);
}
