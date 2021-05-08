package firstEv3TestProgram;


import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;

public class Engine {
	RegulatedMotor left;
	RegulatedMotor right;
	
	public Engine (EV3LargeRegulatedMotor left, EV3LargeRegulatedMotor right) {
		this.left = new EV3LargeRegulatedMotor(MotorPort.A);
		this.right = new EV3LargeRegulatedMotor(MotorPort.B);
	}
	
	public void driveStraightAhead() {
		left.setSpeed(300);
		right.setSpeed(300);
	}
}
