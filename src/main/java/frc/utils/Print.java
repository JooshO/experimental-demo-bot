package frc.utils;

/**
 * Class for printing things but with one more option!
 * If you are debugging, your calls will print. If not, they won't.
 * @author Josh Overbeek
 */
public class Print {
    public synchronized static void dPrintLn(String str) {
        if (Debug.isDebug()) System.out.println(str);
    }

    public synchronized static void dPrintF(String format, Object... args) {
        if (Debug.isDebug()) System.out.printf(format, args);
    }
}