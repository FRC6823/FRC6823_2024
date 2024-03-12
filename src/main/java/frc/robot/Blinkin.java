package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {
	
	private Spark blinkin;

	public Blinkin(int PWM) {
		blinkin = new Spark(PWM);
	}

	public void lightsNormal() {
		blinkin.set(0.93);
	}
}