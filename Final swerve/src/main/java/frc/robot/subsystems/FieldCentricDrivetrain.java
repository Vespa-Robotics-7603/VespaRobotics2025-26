package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class FieldCentricDrivetrain extends FieldCentric {
    private SlewRateLimiter xfilter = new SlewRateLimiter(10);
    private SlewRateLimiter yfilter = new SlewRateLimiter(10); 
    
    // This class modifies withVelocityX and withVelocityY to change in a smoothed manner.

    @Override
    public FieldCentric withVelocityX(double targetvelocity) {
        double filteredvelocity;
        filteredvelocity = (this.VelocityX < targetvelocity) ? xfilter.calculate(targetvelocity) : targetvelocity;
        this.VelocityX = filteredvelocity;
        return this;
    }

    @Override
    public FieldCentric withVelocityY(double targetvelocity) {
        double filteredvelocity;
        filteredvelocity = (this.VelocityY < targetvelocity) ? yfilter.calculate(targetvelocity) : targetvelocity;
        this.VelocityY = filteredvelocity;
        return this;
    }
}
