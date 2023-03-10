package frc.robot.subsystems;

import java.net.NetworkInterface;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    public static class BotPose {
        double m_x, m_y, m_z, m_roll, m_pitch, m_yaw;

        BotPose(double x, double y, double z, double roll, double pitch, double yaw) {
            m_x = x;
            m_y = y;
            m_z = z;
            m_roll = roll;
            m_pitch = pitch;
            m_yaw = yaw;
        }

        public double getX() {
            return m_x;
        }

        public double getY() {
            return m_y;
        }

        public double getZ() {
            return m_z;
        }
    }

    static NetworkTableEntry m_botposeEntry = NetworkTableInstance.getDefault().getTable("limelight")
            .getEntry("botpose");
    // TODO correct pipeline id
    static NetworkTableEntry m_pipelineEntry = NetworkTableInstance.getDefault().getTable("limelight")
            .getEntry("pipeline");

    Vision() {
        m_botposeEntry.getDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    }

    static BotPose getBotPose() {
        double[] botposeRaw = m_botposeEntry.getDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        var botpose = new BotPose(botposeRaw[0], botposeRaw[1], botposeRaw[2], botposeRaw[3], botposeRaw[4], botposeRaw[5]);
        return botpose;
    }

}
