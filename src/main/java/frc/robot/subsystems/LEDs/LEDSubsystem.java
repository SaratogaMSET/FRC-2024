package frc.robot.subsystems.LEDs;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Candles;

public class LEDSubsystem extends SubsystemBase {
    public CANdle led = new CANdle(Candles.led, Constants.CANBus);
    private int _r = 0;
    private int _g = 0;
    private int _b = 255;
    private double startTime = Timer.getFPGATimestamp();
    private int num_leds = 10;
    public enum ColorPositions {
        INTAKE_AMP_MODE,
        INTAKE_GROUND_MODE,
        INTAKE_SPEAKER_MODE
    }

    public LEDSubsystem() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 1.0;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        led.configAllSettings(candleConfiguration);
    }
    public Command setColor(int r, int g, int b) {
        return this.runOnce(() -> led.setLEDs(r, g, b));
    }
    public Color lerpColor(Color a, Color b, double t){
        return new Color((b.red - a.red)*t + a.red, (b.green - a.green)*t + a.green, (b.blue - a.blue)*t + a.blue);
    }

    public void aquamarineAnimation(double periodicity){
        Color aqA = new Color(24, 255, 95);
        Color aqB = new Color(0, 123, 255);
        double time = (Timer.getFPGATimestamp() - startTime) / periodicity;
        for(int i = 0; i < num_leds; i++){
            double percentIndex = i/(num_leds - 1);
            double t = (time + percentIndex) - (int)(time + percentIndex);
            Color gradient = lerpColor(aqA, aqB, t);
            led.setLEDs((int) gradient.red, (int) gradient.green, (int) gradient.blue, 0, i, 1);
        }
    }

}
