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


public class LorteKnep
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
	
	
	static SampleProvider distancefront = sens1UltraFront.getMode("Distance");
	static float[] sampledistfront = new float[distancefront.sampleSize()];
	
	static SampleProvider distanceside = sens2UltraSide.getMode("Distance");
	static float[] sampledistside = new float[distanceside.sampleSize()];
	
	
	
	static int fx = 200;
	static int fy = 0;

	
	public static void main(String[] args) throws InterruptedException
	{
		
		

		StringBuffer sb = new StringBuffer(16);
		
		
		while(true) {
			
			naviBot.addWaypoint(fx,fy);
			naviBot.followPath();
			distanceside.fetchSample(sampledistside, 0);
			distancefront.fetchSample(sampledistfront,0);

			if(sampledistfront[0]<0.2) {
				naviBot.clearPath();
				Point Left = polarHeading(-90,Poseo.getPose());
				naviBot.addWaypoint(Poseo.getPose().getX()+Left.x,Poseo.getPose().getY()+Left.y);
				naviBot.followPath();
				
				if(naviBot.waitForStop()) {
					LCD.clear();
					LCD.drawInt(4, 1, 1);
					distanceside.fetchSample(sampledistside, 0);
					distancefront.fetchSample(sampledistfront,0);
				}
				
				
			} 
			//for some reason, it fucks up the x and y if you use the second if statement.
			/*
			if(sampledistside[0]<0.3) {
				LCD.clear();
				LCD.drawInt(3, 1, 1);
				naviBot.clearPath();
				//Point Straight = polarHeading(0,Poseo.getPose());
				naviBot.addWaypoint(Poseo.getPose().getX()+5f,Poseo.getPose().getY());
				naviBot.followPath();
				if(naviBot.waitForStop()) {
					LCD.clear();
					LCD.drawInt(5, 1, 1);
					distanceside.fetchSample(sampledistside, 0);
					distancefront.fetchSample(sampledistfront,0);
				}
			}
			*/
			
			while(sampledistside[0]<0.3)
			{
				naviBot.clearPath();
				//Point Straight = polarHeading(0,Poseo.getPose());
				naviBot.addWaypoint(Poseo.getPose().getX()+5f,Poseo.getPose().getY());
				naviBot.followPath();

				if(naviBot.waitForStop()) {
					LCD.clear();
					LCD.drawInt(5, 1, 1);
					distanceside.fetchSample(sampledistside, 0);
					distancefront.fetchSample(sampledistfront,0);
				}
				
			}
			
			
			
		}
		
		
		
		
		
		/*
		while(true) {
			
			
			final Pose start = Poseo.getPose();
			
			Waypoint FUCK = new Waypoint(start.getX()+fx,start.getY()+fy);
			Pose temp = new Pose();
			
			
			naviBot.addWaypoint(FUCK);
			naviBot.followPath();
			LCD.clear();
			LCD.drawInt(1, 1, 1);
			
			while(naviBot.isMoving()) {
				distanceside.fetchSample(sampledistside, 0);
				distancefront.fetchSample(sampledistfront,0);
								
				if(sampledistfront[0]<0.2) {
					LCD.clear();
					LCD.drawInt(2, 1, 1);
					naviBot.clearPath();
					temp = Poseo.getPose();
					Point Left = polarHeading(-90,temp);
					naviBot.addWaypoint(temp.getX()+Left.x,temp.getY()+Left.y,temp.getHeading());
					
					naviBot.followPath();
					if(naviBot.waitForStop()) {
						LCD.clear();
						LCD.drawInt(4, 1, 1);
						distanceside.fetchSample(sampledistside, 0);
						distancefront.fetchSample(sampledistfront,0);
					}
					
				} 
				
				if(sampledistside[0]<0.3) {
					LCD.clear();
					LCD.drawInt(3, 1, 1);
					naviBot.clearPath();
					temp = Poseo.getPose();
					naviBot.addWaypoint(temp.getX()+5f,temp.getY(),temp.getHeading());
					
					naviBot.followPath();
					if(naviBot.waitForStop()) {
						LCD.clear();
						LCD.drawInt(5, 1, 1);
						distanceside.fetchSample(sampledistside, 0);
						distancefront.fetchSample(sampledistfront,0);
					}
				}
				
				naviBot.addWaypoint(FUCK);
				naviBot.followPath();
				
			}
			
			
			
		}*/
		
		
		
	}
}

