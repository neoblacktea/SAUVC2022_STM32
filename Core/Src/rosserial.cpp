/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include "rosserial.h"
#include "ros.h"
#include <string>
#include <sstream>
#include "std_msgs/Float64.h"

ros::NodeHandle nh;

std_msgs::Float64 str_msg;
ros::Publisher chatter("chatter", &str_msg);
extern float val1;

void callback(const std_msgs::Float64& msg)
{
    val1 = msg.data;
}

ros::Subscriber<std_msgs::Float64> sub("chatter_lis", callback);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop_pub(float data)
{

	str_msg.data = data;
	chatter.publish(&str_msg);
	nh.spinOnce();
	HAL_Delay(1000);
}

void loop_sub(void)
{
    nh.spinOnce();
}