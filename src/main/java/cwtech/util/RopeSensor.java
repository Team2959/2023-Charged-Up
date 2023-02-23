package cwtech.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;

public class RopeSensor {
    int m_digitalInputPort;
    DigitalInput m_digitalInput;
    int m_debounceCount;
    boolean m_debounceValue;
    boolean m_lastTickValue;
    boolean m_isGoingForward;
    int m_ticks;

    public RopeSensor(int digitalIOPort) {
        m_digitalInputPort = digitalIOPort;
        m_digitalInput = new DigitalInput(digitalIOPort);
        m_lastTickValue = m_digitalInput.get();
    }

    public int getDigitalInputPort() {
        return m_digitalInputPort;
    }

    public void setGoingForward(boolean forward) {
        m_isGoingForward = forward;
    }

    public void setup(Robot robot) {
        robot.addPeriodic(this::update, 0.005, 0.003);
    }

    private void update() {
        boolean value = m_digitalInput.get();

        if(m_debounceValue == m_lastTickValue) {
            // don't continue updating because we see the same value as previous tick
            return;
        }

        if(m_debounceCount == 0) {
            m_debounceValue = value;
            m_debounceCount++;
        } else if(m_debounceCount > 0) {
            m_debounceCount++;
        }

        if(m_debounceCount >= 2) {
            m_lastTickValue = m_debounceValue;
            m_debounceCount = 0;
            m_ticks += m_isGoingForward ? 1 : -1;
        }
    }

    public int getTicks() {
        return m_ticks;
    }


}
