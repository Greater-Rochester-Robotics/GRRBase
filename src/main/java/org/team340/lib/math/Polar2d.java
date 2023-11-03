package org.team340.lib.math;

import edu.wpi.first.math.interpolation.Interpolatable;

public class Polar2d implements Interpolatable<Polar2d> {

    private final double theta;
    private final double r;

    public Polar2d(double theta, double r) {
        this.theta = theta;
        this.r = r;
    }

    public double getTheta() {
        return theta;
    }

    public double getR() {
        return r;
    }

    @Override
    public Polar2d interpolate(Polar2d endValue, double t) {
        return new Polar2d(theta * t + endValue.theta * (1 - t), r * t + endValue.r * (1 - t));
    }
}
