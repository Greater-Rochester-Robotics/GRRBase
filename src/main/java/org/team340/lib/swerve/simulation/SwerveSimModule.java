package org.team340.lib.swerve.simulation;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import org.team340.lib.swerve.SwerveModule;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveAbsoluteEncoder;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;

public class SwerveSimModule extends SwerveModule {

    private final Timer timer = new Timer();
    private double lastTime;
    private double velocity;
    private double distance;
    private double heading;

    public SwerveSimModule(
        SwerveMotor moveMotor,
        SwerveMotor turnMotor,
        SwerveAbsoluteEncoder absoluteEncoder,
        SwerveConfig config,
        SwerveModuleConfig moduleConfig
    ) {
        super(moveMotor, turnMotor, absoluteEncoder, config, moduleConfig);
        timer.start();
        lastTime = timer.get();
    }

    @Override
    public double getVelocity() {
        return velocity;
    }

    @Override
    public double getDistance() {
        return distance;
    }

    @Override
    public double getAngle() {
        return heading;
    }

    @Override
    public boolean setDesiredState(SwerveModuleState state) {
        boolean flip = super.setDesiredState(state);

        double now = timer.get();
        double dt = now - lastTime;
        lastTime = now;

        velocity = state.speedMetersPerSecond * (flip ? -1.0 : 1.0);
        distance += velocity * dt;
        heading = state.angle.getRadians() + (flip ? Math.PI : 0.0);

        return flip;
    }
}
