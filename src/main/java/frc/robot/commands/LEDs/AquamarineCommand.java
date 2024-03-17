package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDSubsystem;

public class AquamarineCommand extends Command{
    LEDSubsystem leds;

    public AquamarineCommand(LEDSubsystem leds){
        this.leds = leds;
    }

    @Override
    public void execute() {
        leds.aquamarineAnimation(3.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}