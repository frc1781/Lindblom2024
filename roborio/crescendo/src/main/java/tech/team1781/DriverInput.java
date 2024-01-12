package tech.team1781;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import tech.team1781.utils.EVector;

public class DriverInput {

    private GenericHID[] mControllers = new GenericHID[] {
        new XboxController(0)
    };

    public enum ControllerSide {
        LEFT, RIGHT
    }

    public EVector getControllerJoyAxis(ControllerSide side, int controllerIndex) {
        var selectedController = (XboxController) mControllers[0];
        EVector ret_val = new EVector();
        if(side == ControllerSide.LEFT) {
            ret_val.x = selectedController.getLeftX();
            ret_val.y = selectedController.getLeftY();
        } else {
            ret_val.x = selectedController.getRightX();
            ret_val.x = selectedController.getRightY();
        }

        return ret_val;
    }
}
