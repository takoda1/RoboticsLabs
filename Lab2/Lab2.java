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
import java.util.ArrayDeque;
import java.util.ArrayList;

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
		
		touch.fetchSample(sample_touch, 0);
		while(sample_touch[0] == 0){
			forward(mA, mB);
			touch.fetchSample(sample_touch, 0);
		}
		stop(mA, mB);
		backward(mA, mB);
		Delay.msDelay(1500);
		stop(mA, mB);
		
		mA.rotate(380);
		
		ultra.fetchSample(sample_sonic,0);
		float distance = sample_sonic[0];
		float[] d = new float[3];
		//LCD.drawInt(distance, 0, 4);
		float desiredDistance = .1f;
		int count = 0;
		ArrayList<Float> running = new ArrayList<Float>(10);
		while(distance < .5){
			LCD.clear();
			int tempCount = 0;
			for(int i = 0; i < 3; i++){
				ultra.fetchSample(sample_sonic, 0);
				d[i] = sample_sonic[0];
				if(d[i] > 1.5 || d[i] < 0){
					
				}
				else{
					distance += d[i];
					tempCount++;
				}
			}
			distance = distance/(float)tempCount;
		
			running.add(distance);
			if(running.size() > 10){
				running.remove(0);
			}
			
			float distance1 = 0;
			for(int i = 0; i < running.size(); i ++){
				distance1 += running.get(i);
			}
			distance1 = distance1/10;
			
			float difference = distance1 - desiredDistance;
			LCD.drawInt((int)(difference * 100), 0, 4);
			LCD.drawInt((int)(distance1 * 100), 0, 2);
			if(distance1 > desiredDistance + .05){
				//difference is between .2, -.1
//				mA.rotate(-30);
//				mB.rotate(30);
//				forward(mA, mB);
//				Delay.msDelay(1500);
//				stop(mA, mB);
				moveLeft(mA, mB);
			}
			else if(distance1 < desiredDistance){
				moveRight(mA, mB);
			}
			else{
				equalMove(mA, mB);
			}
		}
		mA.setSpeed(200);
		mB.setSpeed(200);
		
		stop(mA, mB);
		forward(mA, mB);
		Delay.msDelay(1000);
		stop(mA, mB);
		Delay.msDelay(500);
		
		mB.rotate(360);
		Delay.msDelay(500);
		
		forward(mA, mB);
		Delay.msDelay(7550);
		stop(mA, mB);
//		mA.startSynchronization();
//		//mA.rotate(1433);
//		mA.rotateTo(1433);
//		mA.endSynchronization();
		
//		float a = 0;
//		while(a < -5.0f){
//			for(int i = 0; i < 3; i++){
//				ultra.fetchSample(sample_sonic, 0);
//				d[i] = sample_sonic[0];
//			}
//			distance = (d[0] + d[1] + d[2]) / 3.0f;
//			LCD.drawInt((int)(distance * 100), 0, 2);
//			Delay.msDelay(500);
//		}

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
	
	static void moveLeft(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){

		ma.setSpeed(200);
		mb.setSpeed(300);
		forward(ma, mb);
	}
	static void moveRight(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){

		ma.setSpeed(300);
		mb.setSpeed(200);
		forward(ma, mb);
	}
	static void equalMove(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){

		ma.setSpeed(200);
		mb.setSpeed(200);
		forward(ma, mb);
	}
}
 