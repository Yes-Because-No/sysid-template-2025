package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.ArrayList;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class LinearSubsystem extends SubsystemBase {

    private ArrayList<TalonFX> motors = new ArrayList<TalonFX>();
    private TalonFXConfiguration config = new TalonFXConfiguration();
    

    public LinearSubsystem(Map<Integer, Boolean> motorMap){
            config.Feedback.SensorToMechanismRatio = Constants.ROTATIONS_TO_LINEAR_UNITS;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Constants.CURRENT_LIMIT;
        }

    public void setVoltage(double voltage){
        
    }

     private SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(
            null,
            null,
            null,
            (state)-> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            voltage -> setVoltage(voltage.magnitude()), 
            null, 
            this
    ));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
     }
     
     public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
     }
}
