package tech.team1781;

import java.util.HashMap;
import java.util.HashSet;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import tech.team1781.utils.EVector;

public class DriverInput {

    private GenericHID[] mControllers = new GenericHID[] {
            new Joystick(0)
        };

    private HashMap<String, Event> mClickEvents = new HashMap<>();
    private HashMap<String, Event> mHoldEvents = new HashMap<>();

    private HashMap<String, Boolean> mButtonMap = new HashMap<>();
    private HashSet<String> mPressedButtons = new HashSet<>();

    public enum ControllerSide {
        LEFT, RIGHT
    }

    public void run() {
        updatePressedButtons();
        checkClickEvents();
        checkHoldEvents();
    }

    public EVector getControllerJoyAxis(ControllerSide side, int controllerIndex) {
        var selectedController = (XboxController) mControllers[0];
        EVector ret_val = new EVector();
        if (side == ControllerSide.LEFT) {
            ret_val.x = selectedController.getLeftX();
            ret_val.y = selectedController.getLeftY();
        } else {
            ret_val.x = selectedController.getRightX();
            ret_val.x = selectedController.getRightY();
        }

        return ret_val;
    }

    public EVector getJoystickJoyAxis(int joyIndex) {
        var selectedJoystick = (Joystick) mControllers[1];
        EVector ret_val = new EVector();

        ret_val.x = selectedJoystick.getX();
        ret_val.y = selectedJoystick.getY();

        return ret_val;
    }

    public void addClickListener(int controllerPort, String button, Event event) {
        String key = createKey(controllerPort, button);

        mClickEvents.put(key, event);
    }

    public void addClickListener(int joyPort, int button, Event event) {
        String key = createKey(joyPort, button);

        mClickEvents.put(key, event);
    }

    public void addHoldListener(int controllerPort, String button, Event event) {
        String key = createKey(controllerPort, button);

        mHoldEvents.put(key, event);
    }

    public void addHoldListener(int joyPort, int button, Event event) {
        String key = createKey(joyPort, button);

        mHoldEvents.put(key, event);
    }

    public boolean getButton(int controllerPort, String button) {
        String key = createKey(controllerPort, button);

        return mButtonMap.get(key);
    }

    public boolean getButton(int joyPort, int button) {
        String key = createKey(joyPort, button);

        return mButtonMap.get(key);
    }

    public EVector getTriggerAxis(int controllerPort) {
        XboxController controller = (XboxController) mControllers[controllerPort];

        return EVector.newVector(controller.getLeftTriggerAxis(), controller.getRightTriggerAxis());
    }

    private void checkClickEvents() {
        for (String key : mClickEvents.keySet()) {
            boolean buttonPressed = mButtonMap.get(key);

            if (buttonPressed && !mPressedButtons.contains(key)) {
                mClickEvents.get(key).onPress();
                mPressedButtons.add(key);
            } else if (!buttonPressed && mPressedButtons.contains(key)) {
                mPressedButtons.remove(key);
            }

        }
    }

    private void checkHoldEvents() {
        for (String key : mHoldEvents.keySet()) {
            boolean buttonPressed = mButtonMap.get(key);

            if (buttonPressed) {
                mHoldEvents.get(key).onPress();
            }
        }
    }

    public int getPOV(int controllerPort) {
        return ((XboxController) mControllers[controllerPort]).getPOV();
    }

    private void updatePressedButtons() {
        for (int i = 0; i < mControllers.length; i++) {
            GenericHID selectedController = mControllers[i];

            if (selectedController instanceof XboxController) {
                updateControllerButtons(i, (XboxController) selectedController);
            } else {
                updateJoystickButtons(i, (Joystick) selectedController);
            }

        }
    }

    private void updateControllerButtons(int index, XboxController controller) {
        mButtonMap.put(createKey(index, "LB"), controller.getAButton());
        mButtonMap.put(createKey(index, "RB"), controller.getBButton());
        mButtonMap.put(createKey(index, "Y"), controller.getAButton());
        mButtonMap.put(createKey(index, "X"), controller.getBButton());
        mButtonMap.put(createKey(index, "B"), controller.getAButton());
        mButtonMap.put(createKey(index, "A"), controller.getBButton());
        mButtonMap.put(createKey(index, "START"), controller.getAButton());
        mButtonMap.put(createKey(index, "BACK"), controller.getBButton());
    }

    private void updateJoystickButtons(int index, Joystick joystick) {
        for (int i = 1; i < 13; i++) {
            mButtonMap.put(createKey(index, i), joystick.getRawButton(i));
        }
    }

    // {type}:{port}:{button}
    private String createKey(int controllerPort, String button) {
        StringBuilder builder = new StringBuilder();

        builder.append("controller");
        builder.append(controllerPort);
        builder.append(button);

        return builder.toString();
    }

    // {type}:{port}:{button}
    private String createKey(int joyPort, int button) {
        StringBuilder builder = new StringBuilder();

        builder.append("joystick");
        builder.append(joyPort);
        builder.append(button);

        return builder.toString();
    }

    public interface Event {
        public void onPress();
    }

}
