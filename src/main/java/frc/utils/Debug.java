package frc.utils;

/** 
 * Class for tracking whether or not you are debugging.
 * This is seperate from Java's debugger, which doesn't always work too hot when
 * running code live on the robot
 * @author Josh Overbeek
 */
public class Debug {
    private static boolean DEBUG = true;

    public synchronized static void setDebug(boolean debug) {
        DEBUG = debug;
    }

    public static boolean isDebug() {return DEBUG;}
}
