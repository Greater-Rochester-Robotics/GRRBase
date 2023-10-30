package org.team340.lib.blacklight;

import java.util.MissingResourceException;

/**
 * Config builder for {@link Blacklight}.
 */
public class BlacklightConfig {

    private String name;
    private String devicePath;
    private int height = -1;
    private int width = -1;
    private int autoExposure = 0;
    private int absoluteExposure = 0;
    private int gain = 0;
    private double[] cameraPosition;
    private double errorAmbiguity = -1;
    private double tagSize = -1;
    private String tagFamily;
    private String tagLayoutFileName;
    private int debugTag = -1;
    private double[] fieldSize;
    private double[] fieldMargin;
    private double[] standardDeviations;

    /**
     * Use the Blacklight with the specified name.
     * Blacklight's names are configured on device in the connection config.
     * @param name The name of the Blacklight.
     */
    public BlacklightConfig withName(String name) {
        this.name = name;
        return this;
    }

    /**
     * Gets the configured name of the Blacklight.
     */
    public String getName() {
        return name;
    }

    /**
     * Sets the device path of the camera. Typically this is {@code /dev/video0}.
     * @param devicePath The absolute path of the camera.
     */
    public BlacklightConfig setDevicePath(String devicePath) {
        this.devicePath = devicePath;
        return this;
    }

    /**
     * Gets the configured device path of the camera.
     */
    public String getDevicePath() {
        return devicePath;
    }

    /**
     * Sets the resolution of the camera in pixels.
     * @param height The camera's height.
     * @param width The camera's width.
     */
    public BlacklightConfig setResolution(int height, int width) {
        this.height = height;
        this.width = width;
        return this;
    }

    /**
     * Gets the configured resolution height of the camera.
     */
    public int getHeight() {
        return height;
    }

    /**
     * Gets the configured resolution width of the camera.
     */
    public int getWidth() {
        return width;
    }

    /**
     * Sets settings for the camera's sensor.
     * @param autoExposure Auto exposure setting. A value of {@code 1} works well.
     * @param absoluteExposure The absolute exposure setting. A value of {@code 10} works well.
     * @param gain The gain setting. A value of {@code 25} works well.
     */
    public BlacklightConfig setSensorSettings(int autoExposure, int absoluteExposure, int gain) {
        this.autoExposure = autoExposure;
        this.absoluteExposure = absoluteExposure;
        this.gain = gain;
        return this;
    }

    /**
     * Gets the configured auto exposure setting.
     */
    public int getAutoExposure() {
        return autoExposure;
    }

    /**
     * Gets the configured absolute exposure setting.
     */
    public int getAbsoluteExposure() {
        return absoluteExposure;
    }

    /**
     * Gets the configured gain setting.
     */
    public int getGain() {
        return gain;
    }

    /**
     * Sets the camera's position, relative to the robot's center. Units are meters and radians.
     * @param x The camera's X position in meters.
     * @param y The camera's Y position in meters.
     * @param z The camera's Z position in meters.
     * @param rx The camera's rotational X position (roll) in radians.
     * @param ry The camera's rotational Y position (pitch) in radians.
     * @param rz The camera's rotational Z position (yaw) in radians.
     */
    public BlacklightConfig setCameraPosition(double x, double y, double z, double rx, double ry, double rz) {
        cameraPosition = new double[] { x, y, z, rx, ry, rz };
        return this;
    }

    /**
     * Gets the configured camera position, as an array of {@code [x, y, z, rx, ry, rz]}.
     */
    public double[] getCameraPosition() {
        return cameraPosition;
    }

    /**
     * Sets the error ambiguity threshold for pose estimation when solve PnP is not in use.
     * @param errorAmbiguity The error ambiguity threshold. A value of {@code 0.15} (15%) works well.
     */
    public BlacklightConfig setErrorAmbiguity(double errorAmbiguity) {
        this.errorAmbiguity = errorAmbiguity;
        return this;
    }

    /**
     * Gets the configured error ambiguity threshold.
     */
    public double getErrorAmbiguity() {
        return errorAmbiguity;
    }

    /**
     * Sets properties for the april tags.
     * The tag layout file should follow the following format:
     * <pre>
     *[
     *  {
     *    "id": integer
     *    "x": float
     *    "y": float
     *    "z": float
     *    "rx": float
     *    "ry": float
     *    "rz": float
     *  },
     *  ...
     *]
     * </pre>
     * @param tagSize The size (length) of the tags in meters.
     * @param tagFamily The tag family ({@code 16h5}, {@code 36h11}, etc) in use. This is case sensitive.
     * @param tagLayoutFileName The name of the JSON file in {@code deploy/blacklight} that specifies the layout of tags on the field (do not include {@code .json}).
     */
    public BlacklightConfig setTagProperties(double tagSize, String tagFamily, String tagLayoutFileName) {
        this.tagSize = tagSize;
        this.tagFamily = tagFamily;
        this.tagLayoutFileName = tagLayoutFileName;
        return this;
    }

    /**
     * Gets the configured tag size in meters.
     */
    public double getTagSize() {
        return tagSize;
    }

    /**
     * Gets the configured tag family.
     */
    public String getTagFamily() {
        return tagFamily;
    }

    /**
     * Gets the configured tag layout file name.
     */
    public String getTagLayoutFileName() {
        return tagLayoutFileName;
    }

    /**
     * Sets a tag ID to be used for debugging.
     * @param debugTag The ID of the tag to use for debugging. {@code -1} disables the debug tag.
     */
    public BlacklightConfig setDebugTag(int debugTag) {
        this.debugTag = debugTag;
        return this;
    }

    /**
     * Gets the configured debug tag.
     */
    public int getDebugTag() {
        return debugTag;
    }

    /**
     * Sets the size of the field.
     * @param fieldLength The field's length in meters. Typically {@code 16.5417}.
     * @param fieldWidth The field's width in meters. Typically {@code 8.0136}.
     * @param z If the field has vertical elements the robot can drive onto, this should be set to the highest point in meters. Otherwise, use {@code 0}.
     */
    public BlacklightConfig setFieldSize(double fieldLength, double fieldWidth, double z) {
        fieldSize = new double[] { fieldLength, fieldWidth, z };
        return this;
    }

    /**
     * Gets the configured field size, as an array of {@code [fieldLength, fieldWidth, z]}.
     */
    public double[] getFieldSize() {
        return fieldSize;
    }

    /**
     * Sets margins on the field's outer bounds to still accept vision measurements from.
     * @param lengthMargin The margin for the field's length.
     * @param widthMargin The margin for the field's width.
     * @param zMargin The margin for the field's Z.
     */
    public BlacklightConfig setFieldMargin(double lengthMargin, double widthMargin, double zMargin) {
        fieldMargin = new double[] { lengthMargin, widthMargin, zMargin };
        return this;
    }

    /**
     * Gets the configured field margin, as an array of {@code [lengthMargin, widthMargin, zMargin]}.
     */
    public double[] getFieldMargin() {
        return fieldMargin;
    }

    /**
     * Sets standard deviations for vision measurements.
     * A good starting configuration is all axis with a magnitude of {@code 0.4}.
     * @param x The X axis standard deviation in meters.
     * @param y The Y axis standard deviation in meters.
     * @param rot The rotational standard deviation in radians.
     */
    public BlacklightConfig setStandardDeviations(double x, double y, double rot) {
        this.standardDeviations = new double[] { x, y, rot };
        return this;
    }

    /**
     * Gets the configured standard deviations for vision measurements, as an array of {@code [x, y, rot]}.
     */
    public double[] getStandardDeviations() {
        return standardDeviations;
    }

    /**
     * Verifies the config.
     */
    public void verify() {
        if (name == null) throwMissing("Name");
        if (devicePath == null) throwMissing("Camera Device Path");
        if (height == -1) throwMissing("Height");
        if (width == -1) throwMissing("Width");
        if (cameraPosition == null) throwMissing("Camera Position");
        if (errorAmbiguity == -1) throwMissing("Error Ambiguity");
        if (tagSize == -1) throwMissing("Tag Size");
        if (tagFamily == null) throwMissing("Tag Family");
        if (tagLayoutFileName == null) throwMissing("Tag Layout File Name");
        if (fieldSize == null) throwMissing("Field Size");
        if (fieldMargin == null) throwMissing("Field Margin");
        if (standardDeviations == null) throwMissing("Standard Deviations");
    }

    private void throwMissing(String key) {
        throw new MissingResourceException("Missing value: " + key, this.getClass().getSimpleName(), key);
    }
}
