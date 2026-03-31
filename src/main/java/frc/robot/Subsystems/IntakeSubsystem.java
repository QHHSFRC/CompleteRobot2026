package frc.robot.Subsystems;



import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private SparkFlex motor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController controller;

    public IntakeSubsystem() {
        motor = new SparkFlex(1, MotorType.kBrushless);
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        // Create configuration
        SparkFlexConfig config = new SparkFlexConfig();

        // Set PID gains
        config.closedLoop.p(0.0001);
        config.closedLoop.i(0);
        config.closedLoop.d(0);
        //config.closedLoop.ff(0.0002);
        //config.closedLoop.kF = 0.00017;

        // Current limit and brake mode
        config.smartCurrentLimit(40);
        config.idleMode(SparkFlexConfig.IdleMode.kBrake);

        // Apply config
        motor.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
    }

    // Run intake at target RPM
        public void runVelocity(double rpm) {
        // simple velocity control
        controller.setReference(rpm, SparkFlex.ControlType.kVelocity);
    }

    // Stop intake
    public void stop() {
        motor.set(0);
    }
    //chatgpt speed for intake 

    public void set(double speed) {
    motor.set(speed);
}

    // Get actual RPM
    public double getVelocity() {
        return encoder.getVelocity();
    }
}