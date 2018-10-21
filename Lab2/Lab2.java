package lab2;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
import lejos.hardware.lcd.LCD;

public class Lab2 {
	public static void main(String[] args){
		/*
		 * Takoda Ren 730093699
		 * Noah Sayre 730006807
		 */
		EV3MediumRegulatedMotor mA = new EV3MediumRegulatedMotor(MotorPort.A);
		EV3MediumRegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B);
		EV3UltrasonicSensor us4 = new EV3UltrasonicSensor(SensorPort.S4);
		EV3TouchSensor touch1 = new EV3TouchSensor(SensorPort.S1);
		
		mA.synchronizeWith(new EV3MediumRegulatedMotor[]{mB});
		
		SensorMode ultra = (SensorMode) us4.getDistanceMode();
		SensorMode touch = touch1.getTouchMode();
		float[] sample_sonic = new float[ultra.sampleSize()];
		float[] sample_touch = new float[touch.sampleSize()];
		
		//mA.synchronizeWith(new EV3MediumRegulatedMotor[] {mB});
		mA.setSpeed(200);
		mB.setSpeed(200);
		
		Button.ENTER.waitForPressAndRelease();
		
		ultra.fetchSample(sample_sonic,0);
		float distance = sample_sonic[0];
		float[] d = new float[3];
		//LCD.drawInt(distance, 0, 4);
		float desiredDistance = .1f;
		while(distance < .5){
			LCD.clear();
			for(int i = 0; i < 3; i++){
				ultra.fetchSample(sample_sonic, 0);
				d[i] = sample_sonic[0];
			}
			distance = (d[0] + d[1] + d[2]) / 3.0f;
			
			float difference = distance - desiredDistance;
			LCD.drawInt((int)(difference * 100), 0, 4);
			LCD.drawInt((int)(distance * 100), 0, 2);
			if(distance > desiredDistance + .05){
				//difference is between .2, -.1
				
				mA.setSpeed(200);
				//mB.setSpeed(200 + difference * 50);
				mB.setSpeed(300);
				forward(mA, mB);
			}
			else if(distance < desiredDistance){
				//mA.setSpeed(200 + (difference * -1 * 100));
				mA.setSpeed(300);
				mB.setSpeed(200);
				forward(mA, mB);
			}
			else{
				mA.setSpeed(200);
				mB.setSpeed(200);
				forward(mA, mB);
			}
		}
		
		stop(mA, mB);
		float a = 0;
		while(a < -5){
			for(int i = 0; i < 3; i++){
				ultra.fetchSample(sample_sonic, 0);
				d[i] = sample_sonic[0];
			}
			distance = (d[0] + d[1] + d[2]) / 3.0f;
			LCD.drawInt((int)(distance * 100), 0, 2);
		}

/*		touch1.fetchSample(sample_touch, 0);
		LCD.drawInt((int)(sample_touch[0]), 0, 4);
		
		while(sample_touch[0] == 0){
			forward(mA, mB);
			touch1.fetchSample(sample_touch, 0);
			LCD.clear();
			LCD.drawInt((int)(sample_touch[0]), 0, 4);
			Delay.msDelay(250);
		}
		*/
		
		us4.close();
		touch1.close();
	}
	
	static void forward(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){
		ma.startSynchronization();
		ma.forward();
		mb.forward();
		ma.endSynchronization();
	}
	
	static void stop(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){
		ma.startSynchronization();
		ma.stop(true);
		mb.stop(true);
		ma.endSynchronization();
	}
	
	static void backward(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){
		ma.startSynchronization();
		ma.backward();
		mb.backward();
		ma.endSynchronization();
	}
}
 