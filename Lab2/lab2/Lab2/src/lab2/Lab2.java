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
import java.util.ArrayList;
/*
 * Takoda Ren 730093699
 * Noah Sayre 730006807
 */
public class Lab2 {
	public static void main(String[] args){
		EV3MediumRegulatedMotor mA = new EV3MediumRegulatedMotor(MotorPort.A);
		EV3MediumRegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B);
		EV3UltrasonicSensor usltrasonic = new EV3UltrasonicSensor(SensorPort.S4);
		EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S1);
		
		mA.synchronizeWith(new EV3MediumRegulatedMotor[]{mB});
		
		SensorMode ultrasonicSensor = (SensorMode) usltrasonic.getDistanceMode();
		SensorMode touchSensor = touch.getTouchMode();
		float[] ultrasonicSamples = new float[ultrasonicSensor.sampleSize()];
		float[] touchSamples = new float[touchSensor.sampleSize()];
		
		//mA.synchronizeWith(new EV3MediumRegulatedMotor[] {mB});
		mA.setSpeed(200);
		mB.setSpeed(200);
		
		Button.ENTER.waitForPressAndRelease();
		
		touch.fetchSample(touchSamples, 0);
		while(touchSamples[0] == 0){
			forward(mA, mB);
			touch.fetchSample(touchSamples, 0);
		}
		stop(mA, mB);
		backward(mA, mB);
		Delay.msDelay(1500);
		stop(mA, mB);
		
		mA.rotate(380);
		
		ultrasonicSensor.fetchSample(ultrasonicSamples,0);
		float distance = ultrasonicSamples[0];
		float desiredDistance = .15f;
		
		float sampleVal;
		ultrasonicSensor.fetchSample(ultrasonicSamples, 0);
		sampleVal = ultrasonicSamples[0];
		ArrayList<Float> running = new ArrayList<Float>(10);
		running.add(sampleVal);
		
		while(distance < .5){
			LCD.clear();
			//boolean broken = false;
			float goodReading;
			ArrayList<Float> goodReadings = new ArrayList<Float>(5);
			float[] reads = new float[5];
			int count = 0;
			float lastReading = running.get(running.size()-1);
			boolean gotSample = false;
			for (int i = 0; i < 5; i++) {
				ultrasonicSensor.fetchSample(ultrasonicSamples, 0);
				sampleVal = ultrasonicSamples[0];
				LCD.drawInt((int) (sampleVal * 100), 0, 2);
				reads[i] = sampleVal;
				if((lastReading - 0.2) < sampleVal && (lastReading + 0.2) > sampleVal && sampleVal != Float.POSITIVE_INFINITY) {
					gotSample = true;
					goodReadings.add(sampleVal);
					//LCD.drawInt((int) (sampleVal * 100), 0, 2);
				}
			}
			if(gotSample == false) {
				//stop(mA, mB);
				if (reads[0] == Float.POSITIVE_INFINITY) {
					
				} else {
					break;
				}
				//moveRight(mA, mB, 0);
				//continue;
			} else {
				float total = 0.0f;
				for (int i = 0; i < goodReadings.size(); i++) {
					total += goodReadings.get(i);
				}
				float avg = total / (float) goodReadings.size();
				distance = avg;
				running.add(avg);
				
				LCD.drawInt((int) (distance * 100), 0, 4);
				float difference = distance - desiredDistance;
				LCD.drawInt((int) (difference * 100), 0, 6);
			
				if(distance > desiredDistance){
					moveLeft(mA, mB, difference);
				} else if (distance < desiredDistance){
					moveRight(mA, mB, difference);
				} else {
					equalMove(mA, mB);
				}
			}
			
		}
	
	
		
		mA.setSpeed(200);
		mB.setSpeed(200);
		
		stop(mA, mB);
		forward(mA, mB);
		Delay.msDelay(850);
		stop(mA, mB);
		Delay.msDelay(500);
		
		mB.rotate(335);
		Delay.msDelay(500);
		
		forward(mA, mB);
		Delay.msDelay(7550);
		stop(mA, mB);

		
		usltrasonic.close();
		touch.close();
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
	
	static void moveLeft(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb, float difference){
		//int[] speeds = scaleSpeed(difference);
		//ma.setSpeed(speeds[0]);
		//mb.setSpeed(speeds[1]);
		ma.setSpeed(125);
		mb.setSpeed(200);
		forward(ma, mb);
	}
	static void moveRight(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb, float difference){
		//int[] speeds = scaleSpeed(difference);
		//ma.setSpeed(speeds[0]);
		//mb.setSpeed(speeds[1]);
		ma.setSpeed(200);
		mb.setSpeed(125);
		forward(ma, mb);
	}
	static void equalMove(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){

		ma.setSpeed(125);
		mb.setSpeed(125);
		forward(ma, mb);
	}
	
	static int[] scaleSpeed(float difference) {
		int[] speeds = new int[2];
		int leftSpeed = 150;
		int rightSpeed = 150;
		float scale = 20.0f;
		int speedIncrease;
		if (difference > 0) {
			float increase = (difference * 100) * scale;
			speedIncrease  = (int) increase;
			rightSpeed += speedIncrease;
		} else if(difference < 0) {
			float increase = (difference * 100 * -1) * scale;
			speedIncrease  = (int) increase;
			leftSpeed += speedIncrease;
		}
		speeds[0] = leftSpeed;
		speeds[1] = rightSpeed;
		return speeds;
	}
}