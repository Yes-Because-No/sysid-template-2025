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

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private TalonFX motor1 = new TalonFX(Constants.MOTOR_1ID);
    private TalonFX motor2 = new TalonFX(Constants.MOTOR_2_ID);

    private TalonFXConfigurator motor1Configurator = motor1.getConfigurator();
    private TalonFXConfigurator motor2Configurator = motor2.getConfigurator();

    private CurrentLimitsConfigs currConf = new CurrentLimitsConfigs();
    private MotorOutputConfigs motor1OutputConfigs = new MotorOutputConfigs();
    private MotorOutputConfigs motor2OutputConfigs = new MotorOutputConfigs();
    
    private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();

    private StrictFollower motor1Follower = new StrictFollower(motor1.getDeviceID());

    public Elevator(){

            currConf.SupplyCurrentLimitEnable = true;
            currConf.SupplyCurrentLimit = Constants.CURRENT_LIMIT;

            motor1Configurator.apply(currConf);
            motor2Configurator.apply(currConf);

            motor1OutputConfigs.Inverted = Constants.MOTOR_1_INVERTED;
            motor2OutputConfigs.Inverted = Constants.MOTOR_2_INVERTED;

            

            motor1Configurator.apply(motor1OutputConfigs);
            motor2Configurator.apply(motor2OutputConfigs);

            feedbackConfigs.SensorToMechanismRatio = Constants.ROTATIONS_TO_LINEAR_UNITS;

            motor1Configurator.apply(feedbackConfigs);
            motor2Configurator.apply(feedbackConfigs);

            motor2.setControl(motor1Follower);
        }

    
    private final MutVoltage sysIdVoltage = Volts.mutable(0);
    private final MutDistance sysIdPosition =Meters.mutable(0);
    private final MutLinearVelocity sysIdVelocity = MetersPerSecond.mutable(0);
    private SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(voltage -> setVoltage(voltage.magnitude()), log -> {
                log.motor("Elevator").voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                        .linearPosition(sysIdPosition.mut_replace(getPosition(), Meters))
                        .linearVelocity(sysIdVelocity.mut_replace(getVelocity(), MetersPerSecond));
            }, this));

        
    
    private void setVoltage(double voltage) {
        motor1.setVoltage(voltage);
    }

    private double getVoltage() {
        return motor1.getMotorVoltage().getValueAsDouble();
    }

    private double getPosition() {
        return motor1.getPosition().getValueAsDouble();
    }

    private double getVelocity() {
        return motor1.getVelocity().getValueAsDouble();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
