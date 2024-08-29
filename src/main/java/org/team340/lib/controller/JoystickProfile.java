package org.team340.lib.controller;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.team340.lib.commands.CommandBuilder;
import org.team340.lib.util.Math2;

/**
 * Represents a profile for a controller's joystick generated from {@link JoystickProfiler}.
 */
public class JoystickProfile {

    private InterpolatingDoubleTreeMap profile;

    /**
     * Create a joystick profile.
     * @param profile An interpolating tree map representing points on the profile.
     */
    public JoystickProfile(InterpolatingDoubleTreeMap profile) {
        this.profile = profile;
    }

    /**
     * Gets the joystick's X value.
     * @param rawX The raw X value returned by the joystick.
     * @param rawY The raw Y value returned by the joystick.
     */
    public double getX(double rawX, double rawY) {
        double theta = Math.atan2(rawY, rawX);
        return rawX / profile.get(theta);
    }

    /**
     * Gets the joystick's Y value.
     * @param rawX The raw X value returned by the joystick.
     * @param rawY The raw Y value returned by the joystick.
     */
    public double getY(double rawX, double rawY) {
        double theta = Math.atan2(rawY, rawX);
        return rawY / profile.get(theta);
    }

    /**
     * Loads a profile from a file.
     * @param filePath A file path relevant to the root of the deploy directory.
     */
    public static JoystickProfile fromFile(String filePath) {
        InterpolatingDoubleTreeMap profile = new InterpolatingDoubleTreeMap();

        try {
            File file = new File(Filesystem.getDeployDirectory(), filePath);
            List<List<Double>> points = new ObjectMapper().readValue(file, new TypeReference<List<List<Double>>>() {});

            for (List<Double> point : points) {
                profile.put(point.get(0), point.get(1));
            }
        } catch (Exception e) {
            e.printStackTrace();
            profile.clear();
            profile.put(0.0, 1.0);
        }

        return new JoystickProfile(profile);
    }

    /**
     * Command to profile a controller's joystick and print to console.
     * The printed output can be saved to a JSON file and loaded by {@link JoystickProfile#fromFile(String)}.
     * @param controller Controller to profile.
     * @param xAxis X-Axis of controller joystick.
     * @param yAxis Y-Axis of controller joystick.
     * @param samplePoints Number of points to sample.
     */
    public static Command generateProfile(GenericHID controller, int xAxis, int yAxis, int samplePoints) {
        InterpolatingDoubleTreeMap data = new InterpolatingDoubleTreeMap();
        return new CommandBuilder()
            .onInitialize(() -> data.clear())
            .onExecute(() -> {
                double x = controller.getRawAxis(xAxis);
                double y = controller.getRawAxis(yAxis);
                double theta = Math.atan2(y, x);
                double r = Math.hypot(x, y);
                System.out.println("T: " + Math2.toFixed(theta) + " R: " + Math2.toFixed(r));
                data.put(theta, r);
            })
            .onEnd(() -> {
                List<List<Double>> profile = new ArrayList<List<Double>>();
                for (int i = 0; i < samplePoints; i++) {
                    double theta = ((double) i / (double) samplePoints * Math2.TWO_PI) - Math.PI;
                    profile.add(List.of(theta, data.get(theta)));
                }

                try {
                    String json = new ObjectMapper().writeValueAsString(profile);
                    System.out.println(json);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            })
            .ignoringDisable(true);
    }
}
