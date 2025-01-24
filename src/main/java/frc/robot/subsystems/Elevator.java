package frc.robot.subsystems;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator extends SubsystemBase {
    private int MOTOR_1ID = 0;
    private int MOTOR_2_ID = 1;
    
    private TalonFX motor = new TalonFX(MOTOR_1ID);
    private TalonFX motor2 = new TalonFX(MOTOR_2_ID);

    private StrictFollower motor1Follower = new StrictFollower(motor.getDeviceID());

    private final MutVoltage sysIdVoltage = Volts.mutable(0);
    private final MutDistance sysIdPosition =Meters.mutable(0);
    private final MutLinearVelocity sysIdVelocity = MetersPerSecond.mutable(0);
    private SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(voltage -> setVoltage(voltage.magnitude()), log -> {
                log.motor("Elevator").voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                        .linearPosition(sysIdPosition.mut_replace(getPosition(), Meters))
                        .linearVelocity(sysIdVelocity.mut_replace(getVelocity(), MetersPerSecond));
            }, this));

        
    public Elevator(){
        motor2.setControl(motor1Follower);
    }
    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    private double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    private double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    private double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
