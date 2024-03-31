package frc.robot.subsystems.Shooter;

public class ShooterParameters {
    public static double voltage_to_kRPM(double voltage){
        return 0.508779*voltage-0.148742;
    }
    public static  double kRPM_to_voltage(double kRPM){
        return 1/0.508779*kRPM+0.148742/0.508779;
    }
    public static  double voltage_to_mps(double voltage){
        return 2.18115*voltage-1.59467;
    }
    public static  double mps_to_voltage(double mps){
        return 1.05 * 1/2.18115*mps+1.59467/2.18115;
    }
    public static  double mps_to_kRPM(double mps){
        return voltage_to_kRPM(mps_to_voltage(mps));
    }
}
