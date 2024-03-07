package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {
	
	Spark blinkin;

	public Blinkin() {
		blinkin = new Spark(9);
	}

	public void lightsNormal() {
		blinkin.set(0.93);
	}
}