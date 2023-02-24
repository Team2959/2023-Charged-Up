package cwtech.util;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidV2 {
    Solenoid m_hardware;
    boolean m_cache;

    public SolenoidV2(int channel) {
        m_hardware = new Solenoid(PneumaticsModuleType.REVPH, channel);
        m_cache = m_hardware.get();
    }

    public void set(boolean state) {
        m_cache = state;
        m_hardware.set(state);
    }

    public boolean get() {
        return m_cache;
    }
}
