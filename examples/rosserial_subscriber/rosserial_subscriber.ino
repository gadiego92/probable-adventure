/*
   rosserial Subscriber Example
   Blinks an LED on callback
*/

#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

void messageCb(const std_msgs::Empty &toggle_msg)
{
  // toggle the LED status
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN));
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
