package tech.team1781.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDs extends Subsystem {
    private final int LED_LENGTH = 151;

    private AddressableLED ledController = null;
    private AddressableLEDBuffer ledBuffer = null;
    private int mFocus = 0;

    private int rainbowFirstPixelHue = 1;

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
            ledBuffer = new AddressableLEDBuffer(LED_LENGTH);

            ledController.setLength(ledBuffer.getLength());
            ledController.setData(ledBuffer);
            ledController.start();
        }
    }

    @Override
    public void getToState() {
        if ((ledBuffer == null || ledController == null)) {
            return;
        }

        switch ((ledState) getState()) {
            case NO_NOTE:
                // for (var i = 0; i < ledBuffer.getLength(); i++) {
                // ledBuffer.setRGB(i, 0, 255, 0);
                // }
                // ledController.setData(ledBuffer);
                vwoop();
                break;
            case HAS_NOTE:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 255, 0, 0);
                }
                // ledController.setData(ledBuffer);
                break;
            default:
                rainbow();
                break;
        }

        ledController.setData(ledBuffer);

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

    @Override
    public void disabledPeriodic() {
        rainbow();
        ledController.setData(ledBuffer);
    }

    private void vwoop() {
        final int SPEED = 1;
        final int FOCUS_LENGTH = 10;
        int evenLength = LED_LENGTH - (LED_LENGTH % 2);
        int halfLength = evenLength / 2;

        for (int i = 0; i < halfLength; i++) {
            int diff = Math.abs(i - mFocus);
            if (diff <= FOCUS_LENGTH) {
                ledBuffer.setLED(i, Color.kRed);
            } else {
                ledBuffer.setLED(i, Color.kBlack);
            }

        }

        if (mFocus >= halfLength) {
            mFocus = 0;
        }

        mFocus += SPEED;

    }

    private void rainbow() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {

            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }

        rainbowFirstPixelHue += 3;

        rainbowFirstPixelHue %= 180;
    }

}
