package tech.team1781.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs extends Subsystem{
    private AddressableLED ledController = null;
    private AddressableLEDBuffer ledBuffer = null;

    public LEDs() {
        super("LEDs", ledState.DEFAULT);
    }

    public enum ledState implements SubsystemState {
        HAS_NOTE,
        NO_NOTE,
        DEFAULT
    }

    @Override
    public void genericPeriodic() {

    }

    @Override
    public void init() {
        if (ledController == null) {
            ledController = new AddressableLED(9);
            ledBuffer = new AddressableLEDBuffer(151);

            ledController.setLength(ledBuffer.getLength());
            ledController.setData(ledBuffer);
            ledController.start();
        }

        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 0, 0);
        }
        ledController.setData(ledBuffer);
    }

    @Override
    public void getToState() {
        if ((ledBuffer == null || ledController == null)) { return; }

        switch ((ledState) getState()) {
            case DEFAULT:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    // Sets the specified LED to the RGB values for red
                    ledBuffer.setRGB(i, 128, 0, 0);

                    if (i == 0 || i % 2 == 0) {
                        ledBuffer.setRGB(i, 0, 128, 0);
                    } else {
                        ledBuffer.setRGB(i, 297, 255, 0);
                    }
                }
                ledController.setData(ledBuffer);
                break;
            case NO_NOTE:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 0, 255, 0);
                }
                ledController.setData(ledBuffer);
                break;
            case HAS_NOTE:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 255, 0, 0);
                }
                ledController.setData(ledBuffer);
                break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        return true;
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {

    }
}
