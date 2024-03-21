package tech.team1781.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs extends Subsystem{
    private AddressableLED ledController = null;
    private AddressableLEDBuffer ledBuffer = null;

    private int rainbowFirstPixelHue = 1;

    private boolean blink = false;

    public LEDs() {
        super("LEDs", ledState.DEFAULT);
    }

    public enum ledState implements SubsystemState {
        HAS_NOTE,
        NO_NOTE,
        WARNING,
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
    }

    @Override
    public void getToState() {
        if ((ledBuffer == null || ledController == null)) { return; }

        switch ((ledState) getState()) {
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
            case WARNING:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    if (!blink) {
                        ledBuffer.setRGB(i, 255, 255,0);
                        blink = true;
                    } else {
                        ledBuffer.setRGB(i, 0,0,0);
                        blink = false;
                    }
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

    private void rainbow() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {

            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }

        rainbowFirstPixelHue += 3;

        rainbowFirstPixelHue %= 180;
    }

    @Override
    public void disabledPeriodic() {
        rainbow();
        ledController.setData(ledBuffer);
    }

}
