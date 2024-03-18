package frc.robot.subsystems.Shooter;

public class MajickProfile {
	//Responsive Velocity Controller
	
	//Constraints
	boolean velRanged = false; //If true, return max velocity if above and vice versa for min
	double maxVel;
	double minVel;
	
	//Measure Values
	double time;
	double pTime;
	
	double vel;
	double pVel;
	
	double acc;
	double pAcc;
	
	//For Constructing Profile
	double sign = 1;
	double accLim;
	double jerkLim;
	double targetVel;
	
	//Lines Which Make Profile
	quad test1 = new quad();
	line test2 = new line();
	quad test3 = new quad();
	
	double tp1;
	double tp2;
	double tp3;
	double tp4;
	
	//Shifted To Final Position
	quad first = new quad();
	line second = new line();
	quad third = new quad();
	
	double p1;
	double p2;
	double p3;
	double p4;
	
	
	
	public MajickProfile(double kJ, double kA) {
		this.jerkLim = kJ;
		this.accLim = kA;
		this.targetVel = 0;
	}
	public MajickProfile(double kJ, double kA, double targetVel) {
		this.jerkLim = kJ;
		this.accLim = kA;
		this.targetVel = targetVel;
	}
	
	public void setParams(double currentTime, double currentVel, double currentAcc, double newTarget) {
		sign = Math.abs(sign);
		accLim = Math.abs(accLim);
		jerkLim = Math.abs(jerkLim);
		targetVel = newTarget;
		if(targetVel < currentVel) {
			sign *= -1;
			accLim *= -1;
			jerkLim *= -1;
		}
		
		
		pTime = time;
		pVel = vel;
		pAcc = acc;
		
		time = currentTime;
		vel = currentVel;
		acc = currentAcc;
		
		tp1 = 0;
		
		if(Math.abs(acc) > Math.abs(accLim) && sign(acc) == sign) {
			//Acceleration > Maximum Acceleration while acceleration faces towards target
			test1.setParabVertex(-jerkLim/2, 0, 0);
			test1.setParabVertex(-jerkLim/2, -test1.findDerivative(acc), -test1.findYAtX(test1.findDerivative(acc)) + vel);
		}else if(Math.abs(acc) > Math.abs(accLim) && sign(acc) != sign){
			//Acceleration > Max acceleration while acceleration is opposite of target
			test1.setParabVertex(jerkLim/2, 0, 0);
			test1.setParabVertex(jerkLim/2, -test1.findDerivative(acc), -test1.findYAtX(test1.findDerivative(acc)) + vel);
		}else {
			//Acceleration is under max Acceleration, all is normal
			test1.setParabVertex(jerkLim/2, 0, 0);
			test1.setParabVertex(jerkLim/2, -test1.findDerivative(acc), -test1.findYAtX(test1.findDerivative(acc)) + vel);
		}
		
		//Determine Cases
		double vertexX = test1.vertexX();
		double vertexY = test1.findYAtX(vertexX);
		double prospectX = test1.findDerivative(accLim);
		double prospectY = test1.findYAtX(prospectX);
		
		if((Math.abs(targetVel - vel) < 0.01)) {
			//Velocity is basically same as targetVelocity
			//System.out.println("Case 0");
			test2.setPointSlope(0, 0, targetVel);
			
			tp1 = -1000000;
			tp2 = -1000000;
			tp3 = 1000000;
			tp4 = 1000000; 
		}else if(Math.abs(acc) < Math.abs(accLim) && Math.abs(prospectY - vertexY) * 2 < Math.abs(targetVel - vertexY)) {
			//Case Where Acc < Limit and absolute velocity increases, Cruise Is Reached
			//System.out.println("Case 1 : Accelerate Cruise Decellerate");
			test2.setPointSlope(accLim, test1.findDerivative(accLim), test1.findYAtX(test1.findDerivative(accLim)));
			double yTransition = targetVel - (test1.findYAtX(test1.findDerivative(accLim)) - test1.findYAtX(test1.vertexX()));
			double xVert = test2.findXAtY(yTransition) + Math.abs(test1.findDerivative(accLim) - test1.vertexX());
			
			test3.setParabVertex(-jerkLim/2, xVert, targetVel);
			
			tp2 = test1.findDerivative(accLim);
			tp3 = test3.findDerivative(accLim);
			tp4 = test3.vertexX();
		}else if(Math.abs(acc) < Math.abs(accLim) && Math.abs(prospectY - vertexY) * 2 >= Math.abs(targetVel - vertexY)) {
			//Case Where Acc < Limit, Cruise Is NOT Reached
			//System.out.println("Case 2 : Accelerate Cruise ");
			double xToHalf = sign == 1 ? test1.solve(test1.a, test1.b, test1.c - (vel + targetVel)/2, 2) : test1.solve(test1.a, test1.b, test1.c - (vel + targetVel)/2, 1);
			double xToHalfNAcc = sign == 1 ? test1.solve(test1.a, 0, test1.c - (vel + targetVel)/2, 2) : test1.solve(test1.a, 0, test1.c - (vel + targetVel)/2, 1);
			
			tp2 = tp1 + xToHalf;
			tp3 = tp1 + xToHalf;
			tp4 = tp1 + (xToHalf + xToHalfNAcc);
			
			test3.setParabVertex(-jerkLim/2, (xToHalf + xToHalfNAcc), targetVel);
		}else if(Math.abs(acc) >= Math.abs(accLim) && sign(vel - targetVel) == sign(vertexY - targetVel)) {
			//Case where Acc >= Limit, Cruise Is Reached with no overshoot
			// && Math.abs(prospectY - vertexY) * 2 < Math.abs(targetVel - vertexY) //Perhaps
			//System.out.println("Case 3");
			tp2 = prospectX;
			
			test2.setPointSlope(accLim, tp2, test1.findYAtX(tp2));
			
			tp3 = tp1 + test2.findXAtY(targetVel - Math.abs(vertexY - prospectY) * sign);
			tp4 = tp1 + test2.findXAtY(targetVel - Math.abs(vertexY - prospectY) * sign) + Math.abs(prospectX - vertexX);
			
			test3.setParabVertex(-jerkLim/2, tp4, targetVel);
		}else if(Math.abs(acc) > Math.abs(accLim) && sign(vel - targetVel) != sign(vertexY - targetVel) && Math.abs(prospectY - vertexY) * 2 >= Math.abs(vertexY - targetVel)) {
			//Case where Acc > Limit and overshoot is required to decelerate sufficiently, yet no cruise is required
			//System.out.println("Case 4: Overshoot Without Cruising");
			double xToHalf = test1.solve(test1.a, test1.b, test1.c - (vertexY + targetVel)/2, 1);
			
			if(sign == -1) xToHalf = test1.solve(test1.a, test1.b, test1.c - (vertexY + targetVel)/2, 2);
			
			tp2 = tp1 + xToHalf;
			tp3 = tp1 + xToHalf;
			tp4 = tp1 + (xToHalf * 2 - vertexX);
			
			test3.setParabVertex(jerkLim/2, (xToHalf * 2 - vertexX), targetVel);
		}else if(Math.abs(acc) > Math.abs(accLim) && sign(vel - targetVel) != sign(vertexY - targetVel) && Math.abs(prospectY - vertexY) * 2 < Math.abs(vertexY - targetVel)) {
			//Case where Acc > Limit and overshoot is required to decellerate as well as cruise
			//System.out.println("Case 5: How did we get here?");
			
			tp2 = test1.findDerivative(-accLim);
			
			test2.setPointSlope(-accLim, tp2, test1.findYAtX(tp2));
			
			tp3 = tp1 + test2.findXAtY(targetVel + (vertexY - prospectY));
			tp4 = tp1 + test2.findXAtY(targetVel + (vertexY - prospectY)) + Math.abs(prospectX - vertexX);
			
			test3.setParabVertex(jerkLim/2 * sign(acc / accLim), tp4, targetVel);
		}else {
//			System.out.println("No Case LOL");
//			
//			if(Math.abs(acc) < Math.abs(accLim)) {
//				System.out.println("Acceleration in bounds");
//			}else {
//				System.out.println("!!! Acceleration out of Bounds");
//			}
//			if(sign(vel - targetVel) != sign(vertexY - targetVel)) {
//				System.out.println("!!! Overshoots");
//			}else {
//				System.out.println("No Overshoot");
//			}
//			
//			//System.out.println(Math.abs(vertexY) < Math.abs(prospectY));
//			if(Math.abs(prospectY - vertexY) * 2 < Math.abs(targetVel - vertexY)) {
//				System.out.println("Velocity Change while non constant acceleration is NOT 100% of the profile");
//			}else {
//				System.out.println("Velocity Change while non constant acceleration IS 100% of the profile");
//			}
//			System.out.println(Math.abs(prospectY - vertexY) * 2 < Math.abs(targetVel - vertexY));
//			
//			System.out.println(prospectY);
//			System.out.println(prospectX);
//			System.out.println(vertexY);
//			
//			
//			System.out.println("");
//			System.out.println(2 * Math.abs(prospectY - vertexY));
//			System.out.println(Math.abs(vertexY - targetVel));
		}
		
		//Offset for time ------>
		
		first.setParabVertex(test1.a, test1.vertexX() + currentTime, test1.findYAtX(test1.vertexX()));
		second.setPointSlope(test2.m, 0 + currentTime, test2.findYAtX(0));
		third.setParabVertex(test3.a, test3.vertexX() + currentTime, test3.findYAtX(test3.vertexX()));
		
		p1 = tp1 + currentTime;
		p2 = tp2 + currentTime;
		p3 = tp3 + currentTime;
		p4 = tp4 + currentTime;
	}
	
	double returnVelocity(double x) {
		if(x < p1) {
			return vel;
		}else if(x >= p1 && x < p2) {
			if(first.findYAtX(x) > maxVel && velRanged) {
				return maxVel;
			}else if(first.findYAtX(x) < minVel && velRanged) {
				return minVel;
			}else {
				return first.findYAtX(x);
			}
		}else if(x >= p2 && x < p3) {
			if(first.findYAtX(x) > maxVel && velRanged) {
				return maxVel;
			}else if(first.findYAtX(x) < minVel && velRanged) {
				return minVel;
			}else {
				return second.findYAtX(x);
			}
		}else if(x >= p3 && x < p4) {
			if(first.findYAtX(x) > maxVel && velRanged) {
				return maxVel;
			}else if(first.findYAtX(x) < minVel && velRanged) {
				return minVel;
			}else {
				return third.findYAtX(x);
			}
		}else {
			if(first.findYAtX(x) > maxVel && velRanged) {
				return maxVel;
			}else if(first.findYAtX(x) < minVel && velRanged) {
				return minVel;
			}else {
				return targetVel;
			}
		}
	}
	
	//kV * velocity + kA * acceleration + kPV * (measuredVelocity - velocity) + kPP * (measuredPosition - position)
	
	void printTestProfile() {
		System.out.println(test1.a + "x^{2} + " + test1.b + "x + " + test1.c + " = y \\left\\{" + tp1 + "<x<" + tp2 + "\\right\\}"); //x^{2}
		System.out.println("y = " + test2.m + "(x - " + 0 + ") + " + test2.b + " \\left\\{" + tp2 + "<x<" + tp3 + "\\right\\}");
		System.out.println(test3.a + "x^{2} + " + test3.b + "x + " + test3.c + " = y \\left\\{" + tp3 + "<x<" + tp4 + "\\right\\}");
	}
	void printProfile() {
		System.out.println(first.a + "x^{2} + " + first.b + "x + " + first.c + " = y \\left\\{" + p1 + "<x<" + p2 + "\\right\\}"); //x^{2}
		System.out.println("y = " + second.m + "(x - " + 0 + ") + " + second.b + " \\left\\{" + p2 + "<x<" + p3 + "\\right\\}");
		System.out.println(third.a + "x^{2} + " + third.b + "x + " + third.c + " = y \\left\\{" + p3 + "<x<" + p4 + "\\right\\}");
	}
	double sign(double input) {
		return Math.abs(input)/input;
	}
	
	class quad {
		double a;
		double b;
		double c;
		
		double x1;
		double x2;
		
		double x1() {
			double termSum = a * Math.pow(x1, 3)/3 + b * Math.pow(x1, 2)/2 + x1 * c;
			return Math.abs(termSum/(-1));
		}
		double x2() {
			double termSum = a * Math.pow(x2, 3)/3 + b * Math.pow(x2, 2)/2 + x2 * c;
			return Math.abs(termSum/(-1));
		}
		double area() {
			return x2() - x1();
		}
		double area(double x1, double x2) {
			this.x1 = x1;
			this.x2 = x2;
			return x2() - x1();
		}
		void setParabSlopeAtPoint(double a, double slope, point K) {
			//derivative = 2a(x-vertex)
			//x - derivative / (2a) = vertex
			double xVert = K.x - slope / (2 * a);
			setParabVertex(a, xVert, K.y + a * Math.pow((K.x - xVert), 2));
		}
		void setParab2Points(point vertex, point reference) {
			double xDiff = reference.x - vertex.x;
			double yDiff = reference.y - vertex.y;
			double a = yDiff/(xDiff * xDiff);
			setParabVertex(a, vertex.x, vertex.y);
		}
		void setParabVertex(double tA, double tX, double tY) {
			//y = a(x-h)^2 + k
			//y = ax^2 - 2hx + h * h + k
			a = tA;
			b = - tA * 2 * tX;
			c = tA * tX * tX + tY;
		}
		void setParab(double tA, double tB, double tC) {
			a = tA;
			b = tB;
			c = tC;
		}
		double vertexX () {
			return -b / (2 * a);
		}
		double derivative(double x) {
			return 2 * a * (x - vertexX());
		}
		double findDerivative(double derivative) {
			return derivative / (2 * a) + vertexX();
		}
		double findYAtX(double x) {
			return a * x * x + b * x + c;
		}
		double solve(double a, double b, double c, int root) {
			if(root == 1) {
				return ((-b - Math.sqrt(b*b-4*a*c))/(2*a));
			}else if(root == 2) {
				return ((-b + Math.sqrt(b*b-4*a*c))/(2*a));
			}else {
				return 1/0;
			}
		}
		double solve(int root) {
			if(root == 1) {
				return ((-b - Math.sqrt(b*b-4*a*c))/(2*a));
			}else if(root == 2) {
				return ((-b + Math.sqrt(b*b-4*a*c))/(2*a));
			}else {
				return 1/0;
			}
		}
		//https://www.mathsisfun.com/calculus/integration-rules.html
	}
	class line{
		double b;
		double m;
		
		double x1;
		double x2;
		
		double x1() {
			double termSum = m * Math.pow(x1, 2)/2 + x1 * b;
			return Math.abs(termSum/(-1));
		}
		double x2() {
			double termSum = m * Math.pow(x2, 2)/2 + x2 * b;
			return Math.abs(termSum/(-1));
		}
		double area() {
			return x2() - x1();
		}
		double area(double x1, double x2) {
			this.x1 = x1;
			this.x2 = x2;
			return x2() - x1();
		}
		void setPointSlope(double tM, double tX, double tY) {
			//y - tY = m (x - tX)
			//y = mx - mtX + tY
			m = tM;
			b = -m * tX + tY;
		}
		void setSlopeIntercept(double tM, double tB) {
			m = tM;
			b = tB;
		}
		double findYAtX(double x) {
			return m * x + b;
		}
		double findXAtY(double y) {
			return (y - b) / m;
		}
		//https://www.mathsisfun.com/calculus/integration-rules.html
	}
	class point{
		point(double x, double y){
			this.x = x;
			this.y = y;
		}
		point(){
			x = 0;
			y = 0;
		}
		double x;
		double y;
	}
}