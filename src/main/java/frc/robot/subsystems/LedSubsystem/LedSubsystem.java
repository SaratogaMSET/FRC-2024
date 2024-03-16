package frc.robot.subsystems.LedSubsystem;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Candles;

public class LedSubsystem extends SubsystemBase {
    private CANdle led = new CANdle(Candles.led);
    private int _r = 0;
    private int _g = 0;
    private int _b = 0;
    private CANdleConfiguration config = new CANdleConfiguration();
    public enum ColorPositions {
        INTAKE_AMP_MODE,
        INTAKE_GROUND_MODE,
        INTAKE_SPEAKER_MODE
    }

    public LedSubsystem() {
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness
        led.configAllSettings(config);
    }
    public Command setColor(ColorPositions position) {
        switch (position) {
            case INTAKE_AMP_MODE:
                _r = 255;
                _g = 0;
                _b = 0;
                break;
            case INTAKE_GROUND_MODE:
                _r = 0;
                _g = 255;
                _b = 0;
                break;
            case INTAKE_SPEAKER_MODE:
                _r = 0;
                _g = 0;
                _b = 255;
        }
        return this.run(() -> led.setLEDs(_r, _g, _b));

    }

}
