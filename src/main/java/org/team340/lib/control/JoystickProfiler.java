package org.team340.lib.control;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import org.team340.lib.commands.CommandBuilder;
import org.team340.lib.math.Math2;
import org.team340.lib.math.Polar2d;

public class JoystickProfiler {

    private final GenericHID controller;
    private final InterpolatingDoubleTreeMap data = new InterpolatingDoubleTreeMap();
    private final int xAxis;
    private final int yAxis;

    public JoystickProfiler(GenericHID controller, int xAxis, int yAxis) {
        this.controller = controller;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
    }

    public void clearData() {
        data.clear();
    }

    public void pollData() {
        double x = controller.getRawAxis(xAxis);
        double y = controller.getRawAxis(yAxis);
        double theta = Math.atan2(y, x);
        double r = Math.hypot(x, y);
        data.put(theta, r);
    }

    public List<Polar2d> generateProfile(int samplePoints) {
        List<Polar2d> profile = new ArrayList<>();

        for (int i = 0; i < samplePoints; i++) {
            double theta = (double) i / (double) samplePoints * Math2.TWO_PI;
            profile.add(new Polar2d(theta, data.get(theta)));
        }
        return profile;
    }

    public static Command profilerCommand(GenericHID controller, int xAxis, int yAxis, int samplePoints) {
        JoystickProfiler profiler = new JoystickProfiler(controller, xAxis, yAxis);
        return new CommandBuilder().onInitialize(profiler::clearData).onExecute(profiler::pollData).onEnd(() -> {});
    }
}
