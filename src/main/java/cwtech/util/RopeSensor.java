package cwtech.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class RopeSensor {
    int m_digitalInputPort;
    DigitalInput m_digitalInput;
    int m_debounceCount;
    boolean m_debounceValue;
    int m_ticks;

    RopeSensor(int digitalIOPort) {
        m_digitalInputPort = digitalIOPort;
        m_digitalInput = new DigitalInput(digitalIOPort);
    }

    int getDigitalInputPort() {
        return m_digitalInputPort;
    }

    private void update() {
        boolean value = m_digitalInput.get();
        if(value == m_debounceValue && m_debounceCount > 0) {
            m_debounceCount++;
        } else {
            m_debounceCount = 1;
            m_debounceValue = value;
        }

        if(m_debounceCount >= 2) {
            m_ticks++;
            m_debounceCount = 0;
        }
    }

    public int getTicks() {
        return m_ticks;
    }


}
