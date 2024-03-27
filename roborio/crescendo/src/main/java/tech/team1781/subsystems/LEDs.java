package tech.team1781.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDs extends Subsystem {
    private final int LED_LENGTH = 151;

    private AddressableLED mLedController = null;
    private AddressableLEDBuffer mLedBuffer = null;

    private int mFocus = 0;
    private int mRainbowFirstPixelHue = 1;
    private boolean mFlashMode = false;
    private Timer mTimer = new Timer();

    private boolean blink = false;

    public LEDs() {
        super("LEDs", LedState.DEFAULT);
    }

    public enum LedState implements SubsystemState {
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
        if (mLedController == null) {
            mLedController = new AddressableLED(9);
            mLedBuffer = new AddressableLEDBuffer(LED_LENGTH);

            mLedController.setLength(mLedBuffer.getLength());
            mLedController.setData(mLedBuffer);
            mLedController.start();
        }
    }

    @Override
    public void getToState() {
        if ((mLedBuffer == null || mLedController == null)) {
            return;
        }

        switch ((LedState) getState()) {
            case NO_NOTE:
                // vwoop(Color.kRed);
                break;
            case HAS_NOTE:
                // flashThenSolid(Color.kGreen);
                break;
            default:
                rainbow();
                break;
            case WARNING:
                for (var i = 0; i < mLedBuffer.getLength(); i++) {
                    if (!blink) {
                        mLedBuffer.setRGB(i, 255, 255,0);
                        blink = true;
                    } else {
                        mLedBuffer.setRGB(i, 0,0,0);
                        blink = false;
                    }
                }
                mLedController.setData(mLedBuffer);
                break;
        }

        mLedController.setData(mLedBuffer);

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
        mLedController.setData(mLedBuffer);
    }

    @Override 
    public void setDesiredState(SubsystemState desiredState) {
        if(desiredState == currentState) {
            return;
        }

        super.setDesiredState(desiredState);

        mTimer.reset();
        mTimer.start();
        mFlashMode = false;
    }

    private void flashThenSolid(Color color) {
        final double FLASH_TIME = 1;
        final double BLINK_INTERVAL = 0.25;

        if(mTimer.get() > FLASH_TIME) {
            for(int i = 0; i < mLedBuffer.getLength(); i++) {
                mLedBuffer.setLED(i, color);
            }
            return;
        }

        if(mTimer.get() % BLINK_INTERVAL < BLINK_INTERVAL / 2) {
            for(int i = 0; i < mLedBuffer.getLength(); i++) {
                mLedBuffer.setLED(i, Color.kBlack);
            }
        } else {
            for(int i = 0; i < mLedBuffer.getLength(); i++) {
                mLedBuffer.setLED(i, color);
            }
        }

    }

    private void vwoop(Color color) {
        final int SPEED = 1;
        final int FOCUS_LENGTH = 10;
        int evenLength = LED_LENGTH - (LED_LENGTH % 2);
        int halfLength = evenLength / 2;

        for (int i = 0; i < halfLength; i++) {
            int diff = Math.abs(i - mFocus);
            int otherSide = LED_LENGTH - i;
            if (diff <= FOCUS_LENGTH) {
                mLedBuffer.setLED(i, color);
                mLedBuffer.setLED(otherSide, color);
            } else {
                mLedBuffer.setLED(i, Color.kBlack);
                mLedBuffer.setLED(otherSide, Color.kBlack);
            }

        }

        if (mFocus >= halfLength) {
            mFocus = 0;
        }

        mFocus += SPEED;

    }

    private void rainbow() {
        for (var i = 0; i < mLedBuffer.getLength(); i++) {

            final var hue = (mRainbowFirstPixelHue + (i * 180 / mLedBuffer.getLength())) % 180;
            mLedBuffer.setHSV(i, hue, 255, 128);
        }

        mRainbowFirstPixelHue += 3;

        mRainbowFirstPixelHue %= 180;
    }

}
