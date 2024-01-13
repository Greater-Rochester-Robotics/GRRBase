package org.team340.lib.controller;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.List;
import org.team340.lib.util.Polar2d;

public class JoystickProfile {

    private InterpolatingDoubleTreeMap profile;

    private JoystickProfile(InterpolatingDoubleTreeMap profile) {
        this.profile = profile;
    }

    public double getX(double rawX, double rawY) {
        double theta = Math.atan2(rawY, rawX);
        return rawX / profile.get(theta);
    }

    public double getY(double rawX, double rawY) {
        double theta = Math.atan2(rawY, rawX);
        return rawY / profile.get(theta);
    }

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

    public static JoystickProfile fromPolar2dList(List<Polar2d> list) {
        InterpolatingDoubleTreeMap profile = new InterpolatingDoubleTreeMap();
        for (Polar2d point : list) {
            profile.put(point.getTheta(), point.getR());
        }

        return new JoystickProfile(profile);
    }
}
