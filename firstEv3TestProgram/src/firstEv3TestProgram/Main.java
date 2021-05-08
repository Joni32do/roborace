package firstEv3TestProgram;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;

public class Main {
	

	
	public static void main(String[] args) {
		System.out.println("Ev3");
		EV3LargeRegulatedMotor left = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor right = new EV3LargeRegulatedMotor(MotorPort.B);
		Engine engine1 = new Engine(left, right);
		
		engine1.driveStraightAhead();
	}

}
