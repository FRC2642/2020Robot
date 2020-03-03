package frc.robot.util.library.simple;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class RightSight extends DigitalInput {

    public boolean inverted = false;

    public RightSight(int channel) {
        super(Constants.kRightSight);
    }

    public void setInverted(final boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public boolean get() {
        final boolean val = super.get();
        if (inverted) {
            return !val;
        } else {
            return val;
        }
    }

}