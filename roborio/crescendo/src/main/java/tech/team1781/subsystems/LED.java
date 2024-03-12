package tech.team1781.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch.Type;
import tech.team1781.ConfigMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import tech.team1781.control.ControlSystem;
import tech.team1781.subsystems.Scollector;


public class LED {
    AddressableLED mLed;
    AddressableLEDBuffer mLedBuffer;
    boolean hasNoteScollector = ControlSystem.mScollector.hasNote();

    public void LEDinit() {
        mLed = new AddressableLED(0);
        mLedBuffer = new AddressableLEDBuffer(60);

        mLed.setLength(mLedBuffer.getLength());
        mLed.setData(mLedBuffer);
        mLed.start();
    }

    public void LEDupdate() {

    }

}
