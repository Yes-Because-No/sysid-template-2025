package frc.robot;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    /**Set the current limit to the mechanism's specified amount in the original code */
    public static final double CURRENT_LIMIT = 0;
    /**How much is your mechanism moving if motor spins once? */
    public static final double ROTATIONS_TO_LINEAR_UNITS = 0;
    /**What is the maximum voltage (both forwards and back) for your mechanism? */
    public static final double MAX_VOLTAGE = 12;
    /**
     * What is the ramp rate for your quasistatic test? null = 1Volt/s. 
     * Try making this by Volt.of(#).per(Second)
     * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
     */
    public static final Velocity<VoltageUnit> RAMP_RATE = null;
    /**
     * What is the step voltage for your dynamic test? null = 7 volts
     * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
     */
    public static final Voltage STEP_VOLTAGE = null;
    /**
     * How long will you run your test? null = 15s
     */
    public static final Time TIME_OUT = null;
    //needs at least one entry (pls no break twin)
    /**
     * Add your motor entries here. Within the brackets, for each new motor, add a line with
     * {@code put(CANID, InvertedValue.YourInversionHere)}
     */
    public static final Map<Integer, InvertedValue> MOTORS = new HashMap<>(){{
        put(0, InvertedValue.Clockwise_Positive);
    }};
}
