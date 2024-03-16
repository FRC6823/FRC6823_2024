package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {
	
	private Spark blinkin;

	public Blinkin(int PWM) {
		blinkin = new Spark(PWM);
	}

	public void setLightsNormal() {
		blinkin.set(0.93); //white
	}

	public void setIntakeFull() {
		blinkin.set(0.65); //orange
	}

	public void setIntakeEmpty() {
		setLightsNormal();
	}

}