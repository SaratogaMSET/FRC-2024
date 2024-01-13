package frc.robot.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command{
    DoubleSupplier speed1;
    DoubleSupplier speed2;
    Shooter shooter;
    public ShooterCommand(Shooter shooter, DoubleSupplier speed1, DoubleSupplier speed2){
        this.shooter = shooter;
        this.speed1 = speed1;
        this.speed2 = speed2;
        addRequirements(shooter);
    }
    @Override
    public void execute() {
        shooter.run(speed1.getAsDouble(), speed2.getAsDouble());
    }
}
