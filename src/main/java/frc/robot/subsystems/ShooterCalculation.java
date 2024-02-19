package frc.robot.subsystems;

public class ShooterCalculation {
    private final double g = 9.806;
    public final double epsilon = 0.0009765625;
    private final double epsilon_jacobian = 0.03125;

    private double alpha = 0.001;
    private int maxIters = 25;
    private double tolerance = Math.pow(10, -7);

    public double targetX;
    public double targetY;
    public double targetZ;
    
    public double robotX;
    public double robotY;
    public double robotZ;

    public double robotVX;
    public double robotVY;
    
    public double vMag;

    public boolean isRedSide;
    public void setState(double robotX, double robotY, double robotZ, double robotVX, double robotVY, double vMag, boolean isRedSide){
        this.isRedSide = isRedSide;
        if(isRedSide){
            targetX = 0.0;
            targetY = 0.0;
            targetZ = 2.0;
        }else{
            targetX = 10.0;
            targetY = 0.0;
            targetZ = 2.0;
        }

        this.robotX = robotX;
        this.robotY = robotY;
        this.robotZ = robotZ;

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
        double c1 = Math.pow((targetX - robotX) - (robotVX + vMag * Math.cos(phi) * Math.cos(theta))*t, 2);
        double c2 = Math.pow((targetY - robotZ) - (robotVY + vMag * Math.sin(phi) * Math.cos(theta))*t, 2);
        double c3 = Math.pow((targetZ - robotZ) - (vMag*Math.cos(theta)*t - g*t*t/2), 2);
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
        // System.out.println("TInit:" + Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2) + Math.pow(targetZ - robotZ, 2)));

        //IMPROVED INITIAL GUESS FROM CREATING A VIRTUAL TARGET AND APPLYING https://www.desmos.com/calculator/0k8k97jbwz
        double d = Math.hypot(targetX - robotX, targetY - robotY);
        double naiveFlightTime = 0.35 + (9 - d) * 0.5;
        double virtualTX = targetX - robotVX * naiveFlightTime;
        double virtualTY = targetY - robotVY * naiveFlightTime;
        double virtualD = Math.hypot(virtualTX - robotX, virtualTY - robotY);
        double phi = Math.atan2(virtualTX - robotY, virtualTY - robotX);

        double theta = Math.atan(
            (   vMag*vMag   -   Math.sqrt(  vMag*vMag*vMag*vMag -   g * (   g*virtualD*virtualD  +   2*(targetZ-robotZ)*vMag*vMag   ) )) / 
            (g*virtualD)
            );
        
        return solveShot(
            phi,
            theta,
            naiveFlightTime
        );
    }
    public double[] solveShot(double initialPhi, double initialTheta, double initialT){
        double phi = initialPhi;
        double theta = initialTheta;
        double t = initialT;

        double previousObjective = 999999999;

        for(int i = 0; i < maxIters; i++){
            if(equalityCost(phi, theta, t) < tolerance){
                //NOT RETURNING UNLESS PROBLEM SOLVED, ZERO GRADIENT ISN'T GOOD ENOUGH
                return new double[]{phi, theta, t};
            }
            double[] gradient = gradient(phi, theta, t);

            // System.out.println("Iter " + i + "," + phi + " " + theta + " " + t);
            // System.out.println("Grad: "+ gradient[0] + " " + gradient[1] + " " + gradient[2]);
            
            phi -= alpha * gradient[0];
            theta -= alpha * gradient[1];
            t -= alpha * gradient[2];

            if(t < 0) t = 0.7;

            double currentObjective = constraintFunction(phi, theta, t);
            if(currentObjective < previousObjective){
                alpha *= 1.1;
            }else{
                alpha *= 0.5;
            }
            // System.out.println("Cur: " + currentObjective + ", Pre:" + previousObjective);
            previousObjective = currentObjective;
        }
        // return null;
        return new double[]{phi, theta, t}; //Return failed solve for fun?
    }
    
    /**
     * Solves two shoot on move instances to get derivatives via finite differences
     *
     * @return double[] of {phi, theta, dPhi, dTheta}.
     */
    public double[] solveAll(){
        // System.out.println("------COLD START-------");
        double originalRX = robotX;
        double originalRY = robotY;

        setState(originalRX, originalRY , robotZ, robotVX, robotVY, vMag, isRedSide);
        double[] standard = solveShot();

        setState(originalRX + (epsilon_jacobian * robotVX), originalRY + (epsilon_jacobian * robotVY), robotZ, robotVX, robotVY, vMag, isRedSide);
        double[] plus = solveShot(standard[0], standard[1], standard[2]);
        
        setState(originalRX, originalRY , robotZ, robotVX, robotVY, vMag, isRedSide);

        return new double[]{standard[0], standard[1], standard[2], (plus[0]-standard[0])/(epsilon_jacobian), (plus[1]-standard[1])/(epsilon_jacobian)};
    }
    public double[] solveWarmStart(double initialPhi, double initialTheta, double initialT){
        // System.out.println("------WARM START-------");
        double originalRX = robotX;
        double originalRY = robotY;

        setState(originalRX, originalRY , robotZ, robotVX, robotVY, vMag, isRedSide);
        double[] standard = solveShot(initialPhi, initialTheta, initialT);

        setState(originalRX + (epsilon_jacobian * robotVX), originalRY + (epsilon_jacobian * robotVY), robotZ, robotVX, robotVY, vMag, isRedSide);
        double[] plus = solveShot(standard[0], standard[1], standard[2]);
        
        setState(originalRX, originalRY , robotZ, robotVX, robotVY, vMag, isRedSide);

        return new double[]{standard[0], standard[1], standard[2], (plus[0]-standard[0])/(epsilon_jacobian), (plus[1]-standard[1])/(epsilon_jacobian)};
    }
    public double[] simulateShot(double phi, double theta, double t){
        double x = (robotVX + vMag * Math.cos(phi) * Math.cos(theta))*t;
        double y = (robotVY + vMag * Math.sin(phi) * Math.cos(theta))*t;
        double z = vMag*Math.cos(theta)*t - g*t*t/2;
        return new double[]{x, y, z};
    }
    public boolean shotZone(){ //TODO: Fill zone commands out with conditions
        if(isRedSide){

        }else{

        }
        return false;
    }
    public boolean shotWindupZone(){
        if(isRedSide){

        }else{

        }
        return false;
    }
}
