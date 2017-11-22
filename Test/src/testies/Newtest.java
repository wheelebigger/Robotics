package testies;


import com.sun.corba.se.impl.oa.poa.ActiveObjectMap.Key;
import java.math.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.HiTechnicCompass;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.chassis.*;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.geometry.Point;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.navigation.*;
import lejos.robotics.localization.*;
import lejos.robotics.localization.CompassPoseProvider.*;
import lejos.robotics.navigation.Move;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.Button;

public class Newtest
{
	static RegulatedMotor leftm = new EV3LargeRegulatedMotor(MotorPort.A); //motor 1
	static RegulatedMotor rightm = new EV3LargeRegulatedMotor(MotorPort.B); //motor 2
	
	static Wheel wheelLeft = WheeledChassis.modelWheel(leftm,4.32).offset(-6.35);
	static Wheel wheelRight = WheeledChassis.modelWheel(rightm,4.32).offset(6.35);
	static Chassis chassis = new WheeledChassis(new Wheel[]{wheelLeft, wheelRight},WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot Bronchio = new MovePilot(chassis);
	static OdometryPoseProvider Poseo = new OdometryPoseProvider(Bronchio);
	static Navigator naviBot = new Navigator(Bronchio, Poseo); 
	
	
	static Port port2 = LocalEV3.get().getPort("S2"); //ultrasonic
	static SensorModes sens1UltraFront = new EV3UltrasonicSensor(port2);
	
	static Port port3 = LocalEV3.get().getPort("S3"); //ultrasonic
	static SensorModes sens2UltraSide = new EV3UltrasonicSensor(port3);
	
	static Point polarHeading(int heading, Pose thePose)
	{
		
		float fucklort = Float.parseFloat(String.valueOf(Math.toRadians(thePose.getHeading()+heading)));
		float x = Float.parseFloat(String.valueOf(Math.cos(fucklort)));
		float y = Float.parseFloat(String.valueOf(Math.sin(fucklort)));
		Point polarPoint = new Point(x,y);
		return polarPoint;
	}
	
	/*
	static Port port1 = LocalEV3.get().getPort("S1"); //compass
	static SensorModes sens1Comp = new HiTechnicCompass(port1);
	*/
	
	public static void main(String[] args) throws InterruptedException
	{
		
		SampleProvider distancefront = sens1UltraFront.getMode("Distance");
		float[] sampledistfront = new float[distancefront.sampleSize()];
		
		SampleProvider distanceside = sens2UltraSide.getMode("Distance");
		float[] sampledistside = new float[distanceside.sampleSize()];
		
		/*SampleProvider angle = sens1Comp.getMode("Compass");
		float[] sampledeg = new float[angle.sampleSize()];*/
		
		
		StringBuffer sb = new StringBuffer(16);

		int fx = 200;
		int fy = 0;
		
		while(true)
		{
			
			Pose start = new Pose();
			//start = Poseo.getPose();
			Pose newpose = new Pose();
			naviBot.addWaypoint(fx,fy);
			naviBot.followPath();
			
			
			while(naviBot.isMoving())
			{
				distancefront.fetchSample(sampledistfront,0);
				if(sampledistfront[0]<0.2)
				{
					//naviBot.stop();
					naviBot.clearPath();
					newpose = Poseo.getPose();
					
					
					Point turnLeft = polarHeading(-90,newpose);
					Point turnRight = polarHeading(90,newpose);
					
					
					
					naviBot.addWaypoint(newpose.getX()+40f*turnLeft.x,newpose.getY()+40f*turnLeft.y);
					//Delay.msDelay(100);
					naviBot.addWaypoint(start.getX()+fx,start.getY()+fy);
					
					naviBot.followPath();
					Thread.sleep(500);
					
					
					//Delay.msDelay(50000);
					
				}	
			}
			System.exit(0);
							
		}
	
	}

}
