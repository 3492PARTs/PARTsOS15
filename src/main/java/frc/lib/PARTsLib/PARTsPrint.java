package frc.lib.PARTsLib;

public class PARTsPrint {
    public enum Color {
        RESET("\033[0m"),

        BLACK("\033[0;30m"),
        RED("\033[0;31m"),
        BLUE("\033[0;34m"),
        GREEN("\033[0;32m"),
        YELLOW("\033[0;33m"),
        PURPLE("\033[0;35m"),
        CYAN("\033[0;36m"),
        WHITE("\033[0;37m"),

        B_BLACK("\033[1;30m"),
        B_RED("\033[1;31m"),
        B_BLUE("\033[1;34m"),
        B_GREEN("\033[1;32m"),
        B_YELLOW("\033[1;33m"),
        B_PURPLE("\033[1;35m"),
        B_CYAN("\033[1;36m"),
        B_WHITE("\033[1;37m"),

        U_BLACK("\033[4;30m"),
        U_RED("\033[4;31m"),
        U_BLUE("\033[4;34m"),
        U_GREEN("\033[4;32m"),
        U_YELLOW("\033[4;33m"),
        U_PURPLE("\033[4;35m"),
        U_CYAN("\033[4;36m"),
        U_WHITE("\033[4;37m");

        public final String colorCode;
        private Color(String colorCode) {
            this.colorCode = colorCode;
        }
    };

    public static void log(Object message) {
        System.out.println(Color.RESET.colorCode + message.toString());
    }

    public static void log(Object message, Color color) {
        System.out.println(color.colorCode + message.toString() + Color.RESET.colorCode);
    }

    public static void log(Object messageOne, Color colorOne, Object messageTwo, Color colorTwo) {
        System.out.println(colorOne.colorCode + messageOne.toString() + colorTwo.colorCode + messageTwo.toString() + Color.RESET.colorCode);
    }
}
