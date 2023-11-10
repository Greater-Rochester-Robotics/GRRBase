package org.team340.lib.util;

/**
 * Utilities for manipulating strings.
 */
public final class StringUtil {

    private StringUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Converts a string to camel case.
     * @param str The string to convert.
     * @return The converted string.
     */
    public static String toCamelCase(String str) {
        String[] words = str.split("[\\W_]+");
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < words.length; i++) {
            String word = words[i];
            if (i == 0) word = word.isEmpty() ? word : word.toLowerCase(); else word =
                word.isEmpty() ? word : Character.toUpperCase(word.charAt(0)) + word.substring(1).toLowerCase();
            builder.append(word);
        }
        return builder.toString();
    }

    /**
     * Converts a string to pascal case.
     * @param str The string to convert.
     * @return The converted string.
     */
    public static String toPascalCase(String str) {
        String[] words = str.split("[\\W_]+");
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < words.length; i++) {
            String word = words[i];
            builder.append(word.isEmpty() ? word : Character.toUpperCase(word.charAt(0)) + word.substring(1).toLowerCase());
        }
        return builder.toString();
    }
}
