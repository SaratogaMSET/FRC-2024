package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;

public class ShooterCalculation {
    private final double g = 9.806;
    public final double epsilon = 0.0009765625;
    private final double epsilon_jacobian = 0.03125;

    private final double turretDisplacement = -1 * 0.0254;
    private final double pivotDisplacement = -2.5 * 0.0254;
    private final double outputDisplacementX = 3.191 * 0.0254;
    private final double outputDisplacementY = 4.4337 * 0.0254;

    private double alpha = 0.02;
    private int maxIters = 80;
    private double tolerance = Math.pow(10, -10);

    public double targetX;
    public double targetY;
    public double targetZ;
    
    public double robotX;
    public double robotY;
    public double robotZ;
    public double robotTheta;

    public double robotVX;
    public double robotVY;
    
    public double vMag;

    public void setState(double robotX, double robotY, double robotZ, double robotTheta, double robotVX, double robotVY, double vMag){

        Translation3d target = AllianceFlipUtil.apply(FieldConstants.centerSpeakerOpening);

        this.targetX = target.getX();
        this.targetY = target.getY();
        this.targetZ = target.getZ();

        this.robotX = robotX;
        this.robotY = robotY;
        this.robotZ = robotZ;
        this.robotTheta = robotTheta;

        this.robotVX = robotVX;
        this.robotVY = robotVY;

        this.vMag = vMag;
    }
    public void setSolverParams(double alpha, int maxIters, double tolerance){
        this.alpha = alpha;
        this.maxIters = maxIters;
        this.tolerance = tolerance;
    }
    private double equalityCost(double phi, double theta, double t){
        double dX = targetX - (robotX + Math.cos(robotTheta) * turretDisplacement + Math.cos(phi) * pivotDisplacement + Math.cos(phi) * Math.cos(theta) * outputDisplacementX + Math.cos(phi) * Math.sin(theta) * outputDisplacementY);
        double dY = targetY - (robotY + Math.sin(robotTheta) * turretDisplacement + Math.sin(phi) * pivotDisplacement + Math.sin(phi) * Math.cos(theta) * outputDisplacementX + Math.sin(phi) * Math.sin(theta) * outputDisplacementY);
        double dZ = targetZ - (robotZ + Math.cos(theta) * outputDisplacementY + Math.sin(theta) * outputDisplacementX);

        double shotX = (robotVX + vMag * Math.cos(phi) * Math.cos(theta))*t;
        double shotY = (robotVY + vMag * Math.sin(phi) * Math.cos(theta))*t;
        double shotZ = vMag*Math.sin(theta)*t - g*t*t/2;

        double c1 = Math.pow(dX - shotX, 2);
        double c2 = Math.pow(dY - shotY, 2);
        double c3 = Math.pow(dZ - shotZ, 2);
        return c1 + c2 + c3;
    }
    private double constraintFunction(double phi, double theta, double t){
        double h1 = Math.max(-100*t, 0);
        return equalityCost(phi, theta, t) + h1;
    }
    private double[] gradient(double phi, double theta, double t){
        double dPhi = (constraintFunction(phi + epsilon, theta, t) - constraintFunction(phi - epsilon, theta, t))/(2*epsilon);
        double dTheta = (constraintFunction(phi, theta + epsilon, t) - constraintFunction(phi, theta - epsilon, t))/(2*epsilon);
        double dT = (constraintFunction(phi, theta, t + epsilon) - constraintFunction(phi, theta, t - epsilon))/(2*epsilon);
        return new double[]{dPhi, dTheta, dT};
    }
    public double[] solveShot(){
        // System.out.println("Robot: " + robotX + " " + robotY + " " + robotZ);
        // System.out.println("RobotV: " + robotVX + " " + robotVY);
        // System.out.println("Target: " + targetX + " " + targetY + " " + targetZ);
        // System.out.println("PhiInit:" + Math.atan2(targetY - robotY, targetX - robotX));
        // System.out.println("ThetaINit:" +  Math.atan2(targetZ - robotZ, Math.hypot(targetY - robotY, targetX - robotX)));
        // System.out.println("TInit:" + (Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2) + Math.pow(targetZ - robotZ, 2)))/vMag);

        //IMPROVED INITIAL GUESS FROM CREATING A VIRTUAL TARGET AND APPLYING https://www.desmos.com/calculator/0k8k97jbwz
        double naiveFlightTime = 0.5;
        double virtualTX = targetX - robotVX * naiveFlightTime;
        double virtualTY = targetY - robotVY * naiveFlightTime;
        
        double virtualD = Math.hypot(virtualTY - robotY, virtualTX - robotX);
        double phi = Math.atan2(virtualTY - robotY, virtualTX - robotX);

        double theta = Math.atan(
            (vMag*vMag - Math.sqrt( vMag*vMag*vMag*vMag - g * ( g*virtualD*virtualD + 2*(targetZ-robotZ)*vMag*vMag ) ) )
            / 
            (g*virtualD)
            );
        double a = g/2;
        double b = -vMag*Math.sin(theta);
        double c = targetZ - robotZ;

        return solveShot(
            phi,
            theta,
            (-b-Math.sqrt(b*b - 4 * a * c)) / (2*a)
        );
    }
    public double[] solveShot(double initialPhi, double initialTheta, double initialT){
        while(initialPhi < -3.1415) initialPhi += 2 * 3.1415;
        while(initialPhi > 3.1415) initialPhi -= 2 * 3.1415;
        while(initialTheta < -3.1415) initialTheta += 2 * 3.1415;
        while(initialTheta > 3.1415) initialTheta -= 2 * 3.1415;
        
        double phi = initialPhi;
        double theta = initialTheta;
        double t = initialT;

        double previousObjective = 999999999;
        // System.out.println("Init  " + phi + " " + theta + " " + t + " " + " Objective: " + constraintFunction(phi, theta, t));
        for(int i = 0; i < maxIters; i++){
            if(equalityCost(phi, theta, t) < tolerance){
                //NOT RETURNING UNLESS PROBLEM SOLVED, ZERO GRADIENT ISN'T GOOD ENOUGH
                // System.out.println("Iter " + i + "," + phi + " " + theta + " " + t + " Objective: " + constraintFunction(phi, theta, t));
                // System.out.println("");
                return new double[]{phi, theta, t};
            }
            double[] gradient = gradient(phi, theta, t);

            // System.out.println("Iter " + i + "," + phi + " " + theta + " " + t);
            // System.out.println("Grad: "+ gradient[0] + " " + gradient[1] + " " + gradient[2]);
            
            phi -= alpha * gradient[0];
            theta -= alpha * gradient[1];
            t -= alpha * gradient[2];

            if(t < 0) t = 0.7;
            if(theta < 0 || theta > 1.6) theta = 0.785;

            double currentObjective = constraintFunction(phi, theta, t);
            if(currentObjective < previousObjective){
                alpha *= 1.2;
            }else{
                alpha *= 0.4;
            }
            // System.out.println("Cur: " + currentObjective + ", Pre:" + previousObjective);
            previousObjective = currentObjective;
        }
        // return null;
        // System.out.println("Failure " + phi + " " + theta + " " + t + " " + " Objective: " + constraintFunction(phi, theta, t));
        // System.out.println("");
        return new double[]{phi, theta, t}; //Return failed solve for fun?
    }
    
    /**
     * Solves two shoot on move instances to get derivatives via finite differences
     *
     * @return double[] of {phi, theta, t, dPhi, dTheta}.
     */
    public double[] solveAll(){
        // System.out.println("------COLD START-------");
        double originalRX = robotX;
        double originalRY = robotY;

        setState(originalRX, originalRY , robotZ, robotTheta, robotVX, robotVY, vMag);
        double[] standard = solveShot();

        setState(originalRX + (epsilon_jacobian * robotVX), originalRY + (epsilon_jacobian * robotVY), robotZ, robotTheta, robotVX, robotVY, vMag);
        double[] plus = solveShot(standard[0], standard[1], standard[2]);
        
        setState(originalRX, originalRY , robotZ, robotTheta, robotVX, robotVY, vMag);

        // System.out.println("Stnd: " + standard[0] + " " + standard[1] + " " + standard[2]);
        // System.out.println("Plus: " + plus[0] + " " + plus[1] + " " + plus[2]);
        return new double[]{standard[0], standard[1], standard[2], (plus[0]-standard[0])/(epsilon_jacobian), (plus[1]-standard[1])/(epsilon_jacobian)};
    }
    public double[] solveWarmStart(double initialPhi, double initialTheta, double initialT){
        // System.out.println("------WARM START-------");
        double originalRX = robotX;
        double originalRY = robotY;

        setState(originalRX, originalRY , robotZ, robotTheta, robotVX, robotVY, vMag);
        double[] standard = solveShot(initialPhi, initialTheta, initialT);

        setState(originalRX + (epsilon_jacobian * robotVX), originalRY + (epsilon_jacobian * robotVY), robotZ, robotTheta, robotVX, robotVY, vMag);
        double[] plus = solveShot(standard[0], standard[1], standard[2]);
        
        setState(originalRX, originalRY , robotZ, robotTheta, robotVX, robotVY, vMag);

        return new double[]{standard[0], standard[1], standard[2], (plus[0]-standard[0])/(epsilon_jacobian), (plus[1]-standard[1])/(epsilon_jacobian)};
    }
    public double[] simulateShot(double phi, double theta, double t){
        double x = (robotX + Math.cos(robotTheta) * turretDisplacement + Math.cos(phi) * pivotDisplacement + Math.cos(phi) * Math.cos(theta) * outputDisplacementX + Math.cos(phi) * Math.sin(theta) * outputDisplacementY)
                 + (robotVX + vMag * Math.cos(phi) * Math.cos(theta))*t;
        double y = (robotY + Math.sin(robotTheta) * turretDisplacement + Math.sin(phi) * pivotDisplacement + Math.sin(phi) * Math.cos(theta) * outputDisplacementX + Math.sin(phi) * Math.sin(theta) * outputDisplacementY)
                 + (robotVY + vMag * Math.sin(phi) * Math.cos(theta))*t;
        double z = (robotZ + Math.cos(theta) * outputDisplacementY + Math.sin(theta) * outputDisplacementX)
                 + (vMag*Math.sin(theta)*t - g*t*t/2);
        return new double[]{x, y, z};
    }
    public double[] simulateShotVelocity(double phi, double theta, double t){
        double x = robotVX + vMag * Math.cos(phi) * Math.cos(theta);
        double y = robotVY + vMag * Math.sin(phi) * Math.cos(theta);
        double z = vMag*Math.sin(theta) - g*t;
        return new double[]{x, y, z};
    }
    public boolean shotZone(){ //TODO: Fill zone commands out with conditions

        return true;
    }
    public boolean shotWindupZone(){
        return true;
    }
}