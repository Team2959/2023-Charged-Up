package frc.robot.subsystems;

import java.net.NetworkInterface;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    public class BotPose {
        double m_x, m_y, m_z, m_roll, m_pitch, m_yaw;

        BotPose(double x, double y, double z, double roll, double pitch, double yaw) {
            m_x = x;
            m_y = y;
            m_z = z;
            m_roll = roll;
            m_pitch = pitch;
            m_yaw = yaw;
        }
    }

    NetworkTableEntry m_botposeEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");
    // TODO correct pipeline id
    NetworkTableEntry m_pipelineEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline");

    Vision() {
    }

}
