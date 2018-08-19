/*
   rosserial PubSub Example
   Prints "hello world!" and toggles led
*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

int relay1_pin = 2;
int relay2_pin = 3;
int led_pin = 13;


void messageCb( const std_msgs::String& inMsg) {
  nh.loginfo("messageCb()");
  nh.loginfo(inMsg.data);
  String toto = String(inMsg.data);
  //Serial.println("messageCb");
  //Serial.println(inMsg.data);
  //String test= inMsg.data.c_str();
  if (toto == "SUCKER:ON") {
    digitalWrite(led_pin, HIGH);   // blink the led
    digitalWrite(relay1_pin, HIGH);
  }
  if (toto == "SUCKER:OFF") {
    digitalWrite(led_pin, LOW);   // blink the led
    digitalWrite(relay1_pin, LOW);
  }
}

ros::Subscriber<std_msgs::String> sub("toArduino", messageCb );

std_msgs::String outMsg;
//ros::Publisher chatter("chatter", &outMsg);
ros::Publisher pub("fromArduino", &outMsg);

char hello[50] = "HW from arduino!";

void setup()
{
  //Serial.begin(9600);
  pinMode(relay1_pin, OUTPUT);
  pinMode(relay2_pin, OUTPUT);
  digitalWrite(relay1_pin, LOW);
  digitalWrite(relay2_pin, LOW);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.loginfo("setup()");
  nh.advertise(pub);
  nh.subscribe(sub);

}

void loop()

{
  outMsg.data = TimeToString(millis()/1000);
  pub.publish( &outMsg );
  nh.spinOnce();
  delay(250);
}

char* TimeToString(unsigned long t)
{
  
 static char timearray[12];
 long h = t / 3600;
 t = t % 3600;
 int m = t / 60;
 int s = t % 60;
 sprintf(timearray, "%04ld:%02d:%02d", h, m, s);
 return timearray;
}
