package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDSubsystem;

public class RosegoldCommand extends Command{
    LEDSubsystem leds;

    public RosegoldCommand(LEDSubsystem leds){
        this.leds = leds;
    }

    @Override
    public void execute() {
        leds.rosegoldAnimation(3.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}