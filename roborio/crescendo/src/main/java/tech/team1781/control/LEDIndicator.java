package tech.team1781.control;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIndicator {
    private static final int LED_PORT = 0;
    private static final int LED_LENGTH = 60;
    
    private AddressableLED mLed = new AddressableLED(LED_PORT);
    private AddressableLEDBuffer mLedBuffer = new AddressableLEDBuffer(LED_LENGTH);

    private static LedPattern mPattern = LedPattern.RED_POINTER;
    private static Timer mTimer = new Timer();
    
    private boolean mFlashColor = false;
    private int mPointerIndex = 0;
    
    
    public LEDIndicator() {
        setPattern(LedPattern.GOLD_SOLID);
    }

    public static void setPattern(LedPattern pattern) {
        mPattern = pattern;
        mTimer.reset();
        mTimer.start();
    }

    public void update() {
        switch(mPattern) {
            case GREEN_FLASH:
                greenFlash();
                break;
            default:
            break;
        }
    }

    private void greenFlash() {
        if(mTimer.get() >= 5) {
            mTimer.reset();
            mFlashColor = !mFlashColor;
        }
        for (int i = 0; i < LED_LENGTH; i++) {
            Color color = mFlashColor ? Color.kGreen : Color.kBlack;
            mLedBuffer.setLED(i, color);
        }
    }

    private void redPointer() {
        if(mPointerIndex >= LED_LENGTH) {
            mPointerIndex = 0;
        }

        final int POINTER_LENGTH = 10;
        
        for(int i = 0; i < LED_LENGTH; i++) {
            if(Math.abs(i-mPointerIndex) < POINTER_LENGTH) {
                int brightness = map(Math.abs(i-mPointerIndex), 0, POINTER_LENGTH, 255, 0);
                mLedBuffer.setLED(i, new Color(brightness, 0, 0));
            } else {
                mLedBuffer.setLED(i, Color.kBlack);
            }
        }

        mPointerIndex++;
    }

    private int map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
        return (int) (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }








    public enum LedPattern {
        GREEN_FLASH,
        RED_POINTER,
        RED_VWOOP,
        GOLD_SOLID
    }
}
