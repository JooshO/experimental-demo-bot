package frc.utils;

/**
 * Class for printing things but with one more option!
 * If you are debugging, your calls will print. If not, they won't.
 * @author Josh Overbeek
 */
public class Print {
    /**
     * Printline but only prints in debug
     * @param str String to print
     */
    public synchronized static void dPrintLn(String str) {
        if (Debug.isDebug()) System.out.println(str);
    }

    /**
     * Printf but only in debug
     * @param format String formatting
     * @param args Arguements to put in the formatting
     */
    public synchronized static void dPrintF(String format, Object... args) {
        if (Debug.isDebug()) System.out.printf(format, args);
    }
}