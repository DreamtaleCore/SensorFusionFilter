package com.example.dreamtale.sensorfusionfilter.Utils;

/**
 * Created by DreamTale on 2017/3/25.
 */

import java.util.Random;

/** Utility and fast math functions.
 * <p>
 * Thanks to Riven on JavaGaming.org for the basis of sin/cos/floor/ceil.
 * @author Nathan Sweet */
public final class MathUtils {
    static public final double nanoToSec = 1 / 1000000000f;

    // ---
    static public final double double_ROUNDING_ERROR = 0.000001f; // 32 bits
    static public final double PI = 3.1415927f;
    static public final double PI2 = PI * 2;

    static public final double E = 2.7182818f;

    static private final int SIN_BITS = 14; // 16KB. Adjust for accuracy.
    static private final int SIN_MASK = ~(-1 << SIN_BITS);
    static private final int SIN_COUNT = SIN_MASK + 1;

    static private final double radFull = PI * 2;
    static private final double degFull = 360;
    static private final double radToIndex = SIN_COUNT / radFull;
    static private final double degToIndex = SIN_COUNT / degFull;

    /** multiply by this to convert from radians to degrees */
    static public final double radiansToDegrees = 180f / PI;
    static public final double radDeg = radiansToDegrees;
    /** multiply by this to convert from degrees to radians */
    static public final double degreesToRadians = PI / 180;
    static public final double degRad = degreesToRadians;

    static private class Sin {
        static final double[] table = new double[SIN_COUNT];

        static {
            for (int i = 0; i < SIN_COUNT; i++)
                table[i] = (double)Math.sin((i + 0.5f) / SIN_COUNT * radFull);
            for (int i = 0; i < 360; i += 90)
                table[(int)(i * degToIndex) & SIN_MASK] = (double)Math.sin(i * degreesToRadians);
        }
    }

    /** Returns the sine in radians from a lookup table. */
    static public double sin (double radians) {
        return Sin.table[(int)(radians * radToIndex) & SIN_MASK];
    }

    /** Returns the cosine in radians from a lookup table. */
    static public double cos (double radians) {
        return Sin.table[(int)((radians + PI / 2) * radToIndex) & SIN_MASK];
    }

    /** Returns the sine in radians from a lookup table. */
    static public double sinDeg (double degrees) {
        return Sin.table[(int)(degrees * degToIndex) & SIN_MASK];
    }

    /** Returns the cosine in radians from a lookup table. */
    static public double cosDeg (double degrees) {
        return Sin.table[(int)((degrees + 90) * degToIndex) & SIN_MASK];
    }

    // ---

    /** Returns atan2 in radians, faster but less accurate than Math.atan2. Average error of 0.00231 radians (0.1323 degrees),
     * largest error of 0.00488 radians (0.2796 degrees). */
    static public double atan2 (double y, double x) {
        if (x == 0f) {
            if (y > 0f) return PI / 2;
            if (y == 0f) return 0f;
            return -PI / 2;
        }
        final double atan, z = y / x;
        if (Math.abs(z) < 1f) {
            atan = z / (1f + 0.28f * z * z);
            if (x < 0f) return atan + (y < 0f ? -PI : PI);
            return atan;
        }
        atan = PI / 2 - z / (z * z + 0.28f);
        return y < 0f ? atan - PI : atan;
    }

    // ---

    static public Random random = new Random();

    /** Returns a random number between 0 (inclusive) and the specified value (inclusive). */
    static public int random (int range) {
        return random.nextInt(range + 1);
    }

    /** Returns a random number between start (inclusive) and end (inclusive). */
    static public int random (int start, int end) {
        return start + random.nextInt(end - start + 1);
    }

    /** Returns a random number between 0 (inclusive) and the specified value (inclusive). */
    static public long random (long range) {
        return (long)(random.nextDouble() * range);
    }

    /** Returns a random number between start (inclusive) and end (inclusive). */
    static public long random (long start, long end) {
        return start + (long)(random.nextDouble() * (end - start));
    }

    /** Returns a random boolean value. */
    static public boolean randomBoolean () {
        return random.nextBoolean();
    }

    /** Returns true if a random value between 0 and 1 is less than the specified value. */
    static public boolean randomBoolean (double chance) {
        return MathUtils.random() < chance;
    }

    /** Returns random number between 0.0 (inclusive) and 1.0 (exclusive). */
    static public double random () {
        return random.nextDouble();
    }

    /** Returns a random number between 0 (inclusive) and the specified value (exclusive). */
    static public double random (double range) {
        return random.nextDouble() * range;
    }

    /** Returns a random number between start (inclusive) and end (exclusive). */
    static public double random (double start, double end) {
        return start + random.nextDouble() * (end - start);
    }

    /** Returns -1 or 1, randomly. */
    static public int randomSign () {
        return 1 | (random.nextInt() >> 31);
    }

    /** Returns a triangularly distributed random number between -1.0 (exclusive) and 1.0 (exclusive), where values around zero are
     * more likely.
     * <p>
     * This is an optimized version of {@link #randomTriangular(double, double, double) randomTriangular(-1, 1, 0)} */
    public static double randomTriangular () {
        return random.nextDouble() - random.nextDouble();
    }

    /** Returns a triangularly distributed random number between {@code -max} (exclusive) and {@code max} (exclusive), where values
     * around zero are more likely.
     * <p>
     * This is an optimized version of {@link #randomTriangular(double, double, double) randomTriangular(-max, max, 0)}
     * @param max the upper limit */
    public static double randomTriangular (double max) {
        return (random.nextDouble() - random.nextDouble()) * max;
    }

    /** Returns a triangularly distributed random number between {@code min} (inclusive) and {@code max} (exclusive), where the
     * {@code mode} argument defaults to the midpoint between the bounds, giving a symmetric distribution.
     * <p>
     * This method is equivalent of {@link #randomTriangular(double, double, double) randomTriangular(min, max, (min + max) * .5f)}
     * @param min the lower limit
     * @param max the upper limit */
    public static double randomTriangular (double min, double max) {
        return randomTriangular(min, max, (min + max) * 0.5f);
    }

    /** Returns a triangularly distributed random number between {@code min} (inclusive) and {@code max} (exclusive), where values
     * around {@code mode} are more likely.
     * @param min the lower limit
     * @param max the upper limit
     * @param mode the point around which the values are more likely */
    public static double randomTriangular (double min, double max, double mode) {
        double u = random.nextDouble();
        double d = max - min;
        if (u <= (mode - min) / d) return min + (double)Math.sqrt(u * d * (mode - min));
        return max - (double)Math.sqrt((1 - u) * d * (max - mode));
    }

    // ---

    /** Returns the next power of two. Returns the specified value if the value is already a power of two. */
    static public int nextPowerOfTwo (int value) {
        if (value == 0) return 1;
        value--;
        value |= value >> 1;
        value |= value >> 2;
        value |= value >> 4;
        value |= value >> 8;
        value |= value >> 16;
        return value + 1;
    }

    static public boolean isPowerOfTwo (int value) {
        return value != 0 && (value & value - 1) == 0;
    }

    // ---

    static public short clamp (short value, short min, short max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    static public int clamp (int value, int min, int max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    static public long clamp (long value, long min, long max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }


    static public double clamp (double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    // ---

    /** Linearly interpolates between fromValue to toValue on progress position. */
    static public double lerp (double fromValue, double toValue, double progress) {
        return fromValue + (toValue - fromValue) * progress;
    }

    /** Linearly interpolates between two angles in radians. Takes into account that angles wrap at two pi and always takes the
     * direction with the smallest delta angle.
     *
     * @param fromRadians start angle in radians
     * @param toRadians target angle in radians
     * @param progress interpolation value in the range [0, 1]
     * @return the interpolated angle in the range [0, PI2[ */
    public static double lerpAngle (double fromRadians, double toRadians, double progress) {
        double delta = ((toRadians - fromRadians + PI2 + PI) % PI2) - PI;
        return (fromRadians + delta * progress + PI2) % PI2;
    }

    /** Linearly interpolates between two angles in degrees. Takes into account that angles wrap at 360 degrees and always takes
     * the direction with the smallest delta angle.
     *
     * @param fromDegrees start angle in degrees
     * @param toDegrees target angle in degrees
     * @param progress interpolation value in the range [0, 1]
     * @return the interpolated angle in the range [0, 360[ */
    public static double lerpAngleDeg (double fromDegrees, double toDegrees, double progress) {
        double delta = ((toDegrees - fromDegrees + 360 + 180) % 360) - 180;
        return (fromDegrees + delta * progress + 360) % 360;
    }

    // ---

    static private final int BIG_ENOUGH_INT = 16 * 1024;
    static private final double BIG_ENOUGH_FLOOR = BIG_ENOUGH_INT;
    static private final double CEIL = 0.9999999;
    static private final double BIG_ENOUGH_CEIL = 16384.999999999996;
    static private final double BIG_ENOUGH_ROUND = BIG_ENOUGH_INT + 0.5f;

    /** Returns the largest integer less than or equal to the specified double. This method will only properly floor doubles from
     * -(2^14) to (double.MAX_VALUE - 2^14). */
    static public int floor (double value) {
        return (int)(value + BIG_ENOUGH_FLOOR) - BIG_ENOUGH_INT;
    }

    /** Returns the largest integer less than or equal to the specified double. This method will only properly floor doubles that are
     * positive. Note this method simply casts the double to int. */
    static public int floorPositive (double value) {
        return (int)value;
    }

    /** Returns the smallest integer greater than or equal to the specified double. This method will only properly ceil doubles from
     * -(2^14) to (double.MAX_VALUE - 2^14). */
    static public int ceil (double value) {
        return BIG_ENOUGH_INT - (int)(BIG_ENOUGH_FLOOR - value);
    }

    /** Returns the smallest integer greater than or equal to the specified double. This method will only properly ceil doubles that
     * are positive. */
    static public int ceilPositive (double value) {
        return (int)(value + CEIL);
    }

    /** Returns the closest integer to the specified double. This method will only properly round doubles from -(2^14) to
     * (double.MAX_VALUE - 2^14). */
    static public int round (double value) {
        return (int)(value + BIG_ENOUGH_ROUND) - BIG_ENOUGH_INT;
    }

    /** Returns the closest integer to the specified double. This method will only properly round doubles that are positive. */
    static public int roundPositive (double value) {
        return (int)(value + 0.5f);
    }

    /** Returns true if the value is zero (using the default tolerance as upper bound) */
    static public boolean isZero (double value) {
        return Math.abs(value) <= double_ROUNDING_ERROR;
    }

    /** Returns true if the value is zero.
     * @param tolerance represent an upper bound below which the value is considered zero. */
    static public boolean isZero (double value, double tolerance) {
        return Math.abs(value) <= tolerance;
    }

    /** Returns true if a is nearly equal to b. The function uses the default doubleing error tolerance.
     * @param a the first value.
     * @param b the second value. */
    static public boolean isEqual (double a, double b) {
        return Math.abs(a - b) <= double_ROUNDING_ERROR;
    }

    /** Returns true if a is nearly equal to b.
     * @param a the first value.
     * @param b the second value.
     * @param tolerance represent an upper bound below which the two values are considered equal. */
    static public boolean isEqual (double a, double b, double tolerance) {
        return Math.abs(a - b) <= tolerance;
    }

    /** @return the logarithm of value with base a */
    static public double log (double a, double value) {
        return (double)(Math.log(value) / Math.log(a));
    }

    /** @return the logarithm of value with base 2 */
    static public double log2 (double value) {
        return log(2, value);
    }
}