package cwtech.util;

public class Lifetime {
    public enum RobotMode {
        Auto,
        Teleop,
        Disabled,
        Test,
        Simulation,
    };

    static RobotMode m_mode;

    public static void updateMode(RobotMode mode) {
        m_mode = mode;
    }

    public static boolean isTeleop() {
        return m_mode == RobotMode.Teleop;
    }

    public static RobotMode getMode() {
        return m_mode;
    }
}
