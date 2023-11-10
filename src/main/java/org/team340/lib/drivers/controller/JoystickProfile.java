package org.team340.lib.drivers.controller;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileReader;
import org.json.simple.JSONArray;
import org.json.simple.parser.JSONParser;

public class JoystickProfile {

    private InterpolatingDoubleTreeMap profile;

    private JoystickProfile(InterpolatingDoubleTreeMap profile) {
        this.profile = profile;
    }

    public double getX(double rawX, double rawY) {
        double theta = Math.atan2(rawY, rawX);
        return Math.hypot(rawX, rawY) * profile.get(theta) * Math.cos(theta);
    }

    public double getY(double rawX, double rawY) {
        double theta = Math.atan2(rawY, rawX);
        return Math.hypot(rawX, rawY) * profile.get(theta) * Math.sin(theta);
    }

    public static JoystickProfile fromFile(String filePath) {
        InterpolatingDoubleTreeMap profile = new InterpolatingDoubleTreeMap();

        try {
            File file = new File(Filesystem.getDeployDirectory(), filePath + ".json");
            FileReader fileReader = new FileReader(file);

            JSONArray points = (JSONArray) new JSONParser().parse(fileReader);

            for (int i = 0; i < points.size(); i++) {
                JSONArray point = (JSONArray) points.get(i);
                profile.put((double) point.get(1), (double) point.get(0));
            }
        } catch (Exception e) {
            e.printStackTrace();
            profile.clear();
            profile.put(0.0, 1.0);
        }

        return new JoystickProfile(profile);
    }
}
