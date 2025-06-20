#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include "yhs_can_control.h"

#include <fstream>

std::ofstream outfile;


namespace yhs_tool {


CanControl::CanControl()
{
	ros::NodeHandle private_node("~");
	
}


CanControl::~CanControl()
{

}

//
void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
{
	static unsigned char count_1 = 0;

	cmd_mutex_.lock();

	memset(sendData_u_io_,0,8);

	sendData_u_io_[0] = 0xff;
	if(msg.io_cmd_lamp_ctrl)
		sendData_u_io_[0] &= 0xff;
	else sendData_u_io_[0] &= 0xfe;
	if(msg.io_cmd_unlock)
		sendData_u_io_[0] &= 0xff;
	else sendData_u_io_[0] &= 0xfd;

	sendData_u_io_[1] = 0xff;
	if(msg.io_cmd_lower_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfe;
	if(msg.io_cmd_upper_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfd;

	if(msg.io_cmd_turn_lamp == 0)
		sendData_u_io_[1] &= 0xf3;
	if(msg.io_cmd_turn_lamp == 1)
		sendData_u_io_[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendData_u_io_[1] &= 0xfb;

	if(msg.io_cmd_braking_lamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xef;
	if(msg.io_cmd_clearance_lamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xdf;
	if(msg.io_cmd_fog_lamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xbf;

	sendData_u_io_[2] = msg.io_cmd_speaker;

	sendData_u_io_[3] = 0;
	sendData_u_io_[4] = 0;
	sendData_u_io_[5] = 0;

	count_1 ++;
	if(count_1 == 16)	count_1 = 0;

	sendData_u_io_[6] =  count_1 << 4;

	sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

	send_frames_[0].can_id = 0x98C4D7D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_io_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }
		

	cmd_mutex_.unlock();
}

//
void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg)
{
	short linear = msg.ctrl_cmd_linear * 1000;
	short angular = msg.ctrl_cmd_angular * 100;
	int gear = msg.ctrl_cmd_gear;
	static unsigned char count = 0;

	cmd_mutex_.lock();

	memset(sendData_u_vel_,0,8);//调用了 memset 函数，用于将一个名为 sendData_u_vel_ 的数组的前 8 个字节都设置为 0。这个数组是用于存储要发送的数据的

	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & msg.ctrl_cmd_gear);//sendData_u_vel_ 的第一个字节的低 4 位就表示了车辆的档位
	
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((linear & 0x0f) << 4));//sendData_u_vel_ 的第一个字节的高 4 位就表示了车辆的线速度的低 4 位。

	sendData_u_vel_[1] = (linear >> 4) & 0xff; //将 linear 的值右移 4 位后与 0xff 进行按位与运算，然后赋值给 sendData_u_vel_ 的第二个字节。
												//	这样，sendData_u_vel_ 的第二个字节就表示了车辆的线速度的中 8 位。

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (linear >> 12));//函数的第十六行将 linear 的值右移 12 位后与 0x0f 进行按位与运算，然后赋值给 sendData_u_vel_ 的第三个字节的低 4 位。
																		//这样，sendData_u_vel_ 的第三个字节的低 4 位就表示了车辆的线速度的高 4 位


	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));//函数的第十九行将 sendData_u_vel_ 的第三个字节与 angular 的值的低 4 位左移 4 位后进行按位或运算，然后赋值给 sendData_u_vel_ 的第三个字节。
																				//这样，sendData_u_vel_ 的第三个字节的高 4 位就表示了车辆的角速度的低 4 位。

	sendData_u_vel_[3] = (angular >> 4) & 0xff;//将 angular 的值右移 4 位后与 0xff 进行按位与运算，然后赋值给 sendData_u_vel_ 的第四个字节。
												//这样，sendData_u_vel_ 的第四个字节就表示了车辆的角速度的中 8 位。

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));//angular 的值右移 12 位后与 0x0f 进行按位与运算，然后赋值给 sendData_u_vel_ 的第五个字节的低 4 位。
																	//这样，sendData_u_vel_ 的第五个字节的低 4 位就表示了车辆的角速度的高 4 位。

		// 4H   4L      3H     3L     2H    2L     1H   1L    0H     0L  
		//  *   ang4       ang8      ang4  lin4     lin8     lin4   gear
	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel_[6] =  count << 4;// count 的值左移 4 位，然后赋值给 sendData_u_vel_ 的第七个字节的高 4 位，表示发送的消息的计数。
	

	sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

	send_frames_[0].can_id = 0x98C4D1D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_vel_, 8);//调用了 memcpy 函数，用于将 sendData_u_vel_ 数组的内容复制到 send_frames_ 数组的第一个元素的 data 字段，表示发送的消息的数据内容。

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));//调用了 write 函数，用于将 send_frames_ 数组的第一个元素写入到一个名为 dev_handler_ 的设备句柄中，表示将消息发送到 can 总线上。
																			//函数返回一个整数值，表示写入的字节数。

    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}

// 您好，这是 Bing。我可以尝试用我的知识和词汇来解释一下您提供的 C++ 代码。以下是我的解释结果：

// 这段代码是一个类的成员函数，用于处理一个名为 ctrl_cmd 的消息，该消息是一个自定义的结构体类型，包含了一些控制车辆的指令，如线速度、角速度和档位。函数的参数是一个 ctrl_cmd 类型的变量，表示接收到的消息。

// 函数的第一行定义了一个短整型变量 linear，用于存储消息中的 ctrl_cmd_linear 的值乘以 1000，表示车辆的线速度，单位是 mm/s。

// 函数的第二行定义了一个短整型变量 angular，用于存储消息中的 ctrl_cmd_angular 的值乘以 100，表示车辆的角速度，单位是 0.01 rad/s。

// 函数的第三行定义了一个整型变量 gear，用于存储消息中的 ctrl_cmd_gear 的值，表示车辆的档位，取值范围是 0 到 15。

// 函数的第四行定义了一个静态的无符号字符变量 count，用于记录发送的消息的计数。静态变量的特点是只在第一次执行时初始化，之后保持其值不变，直到程序结束。

// 函数的第六行调用了一个互斥锁，用于保证线程安全，防止多个线程同时访问共享的数据。

// 函数的第八行调用了 memset 函数，用于将一个名为 sendData_u_vel_ 的数组的前 8 个字节都设置为 0。这个数组是用于存储要发送的数据的。

// 函数的第十行将 sendData_u_vel_ 的第一个字节与消息中的 ctrl_cmd_gear 的值进行按位或运算，然后赋值给 sendData_u_vel_ 的第一个字节。这样，sendData_u_vel_ 的第一个字节的低 4 位就表示了车辆的档位。

// 函数的第十二行将 sendData_u_vel_ 的第一个字节与 linear 的值的低 4 位左移 4 位后进行按位或运算，然后赋值给 sendData_u_vel_ 的第一个字节。这样，sendData_u_vel_ 的第一个字节的高 4 位就表示了车辆的线速度的低 4 位。

// 函数的第十四行将 linear 的值右移 4 位后与 0xff 进行按位与运算，然后赋值给 sendData_u_vel_ 的第二个字节。这样，sendData_u_vel_ 的第二个字节就表示了车辆的线速度的中 8 位。

// 函数的第十六行将 linear 的值右移 12 位后与 0x0f 进行按位与运算，然后赋值给 sendData_u_vel_ 的第三个字节的低 4 位。这样，sendData_u_vel_ 的第三个字节的低 4 位就表示了车辆的线速度的高 4 位。

// 函数的第十九行将 sendData_u_vel_ 的第三个字节与 angular 的值的低 4 位左移 4 位后进行按位或运算，然后赋值给 sendData_u_vel_ 的第三个字节。这样，sendData_u_vel_ 的第三个字节的高 4 位就表示了车辆的角速度的低 4 位。

// 函数的第二十一行将 angular 的值右移 4 位后与 0xff 进行按位与运算，然后赋值给 sendData_u_vel_ 的第四个字节。这样，sendData_u_vel_ 的第四个字节就表示了车辆的角速度的中 8 位。

// 函数的第二十三行将 angular 的值右移 12 位后与 0x0f 进行按位与运算，然后赋值给 sendData_u_vel_ 的第五个字节的低 4 位。这样，sendData_u_vel_ 的第五个字节的低 4 位就表示了车辆的角速度的高 4 位。

// 函数的第二十六行将 count 的值加 1，如果等于 16，则重置为 0。这样，count 就表示了发送的消息的计数，从 0 到 15 循环。

// 函数的第二十九行将 count 的值左移 4 位，然后赋值给 sendData_u_vel_ 的第七个字节的高 4 位，表示发送的消息的计数。

// 函数的第三十二行将 sendData_u_vel_ 的前 7 个字节进行异或运算，然后赋值给 sendData_u_vel_ 的第八个字节，表示发送的消息的校验位。

// 函数的第三十四行将一个名为 send_frames_ 的数组的第一个元素的 can_id 字段设置为 0x98C4D1D0，表示发送的消息的标识符。

// 函数的第三十五行将一个名为 send_frames_ 的数组的第一个元素的 can_dlc 字段设置为 8，表示发送的消息的数据长度。

// 函数的第三十七行调用了 memcpy 函数，用于将 sendData_u_vel_ 数组的内容复制到 send_frames_ 数组的第一个元素的 data 字段，表示发送的消息的数据内容。

// 函数的第三十九行调用了 write 函数，用于将 send_frames_ 数组的第一个元素写入到一个名为 dev_handler_ 的设备句柄中，表示将消息发送到 can 总线上。函数返回一个整数值，表示写入的字节数。

// 函数的第四十到四十三行判断 write 函数的返回值，如果小于等于 0，表示发送失败，调用 ROS_ERROR 函数打印错误信息。

// 函数的第四十六行调用了一个互斥锁，用于释放锁，结束线程安全的区域。

// 函数的最后一行是一个右花括号，表示函数的结束。

// 源: 与必应的对话， 2023/12/4
// (1) C++ Introduction - W3Schools. https://www.w3schools.com/cpp/cpp_intro.asp.
// (2) Code Explanation - Educative. https://www.educative.io/courses/learn-cpp-from-scratch/code-explanation.
// (3) Hello World Program in C++ with Code Explanation - Guru99. https://www.guru99.com/cpp-hello-world-program.html.







//
void CanControl::free_ctrl_cmdCallBack(const yhs_can_msgs::free_ctrl_cmd msg)
{
	short linearl = msg.free_ctrl_cmd_velocity_l * 1000;
	short linearr = msg.free_ctrl_cmd_velocity_r * 1000;
	static unsigned char count_3 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((linearl & 0x0f) << 4));

	sendData_u_tem_[1] = (linearl >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (linearl >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((linearr & 0x0f) << 4));

	sendData_u_tem_[3] = (linearr >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (linearr >> 12));

	count_3 ++;

	if(count_3 == 16)	count_3 = 0;

	sendData_u_tem_[6] =  count_3 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

	send_frames_[0].can_id = 0x98C4D2D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}


//数据接收解析线程
void CanControl::recvData()
{

	while(ros::ok())
	{

		if(read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
		{
			for(int j=0;j<1;j++)
			{
				
				switch (recv_frames_[0].can_id)
				{
																										// 4H   4L      3H     3L     2H    2L     1H   1L    0H     0L  
																										//  *   ang4       ang8      ang4  lin4     lin8     lin4   gear
					case 0x98C4D1EF:
					{
						yhs_can_msgs::ctrl_fb msg;
						msg.ctrl_fb_target_gear = 0x0f & recv_frames_[0].data[0];
						
						msg.ctrl_fb_linear = (float)((short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
						
						msg.ctrl_fb_angular = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							ctrl_fb_pub_.publish(msg);
						}

						break;
					}

				
					//
					case 0x98C4D7EF:
					{
						yhs_can_msgs::l_wheel_fb msg;
						msg.l_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.l_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							l_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//
					case 0x98C4D8EF:
					{
						yhs_can_msgs::r_wheel_fb msg;
						msg.r_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.r_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							r_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//io反馈
					case 0x98C4DAEF:
					{
						yhs_can_msgs::io_fb msg;
						if(0x01 & recv_frames_[0].data[0]) msg.io_fb_lamp_ctrl = true;	else msg.io_fb_lamp_ctrl = false;
	
						if(0x02 & recv_frames_[0].data[1]) msg.io_fb_unlock = true;	else msg.io_fb_unlock = false;

						if(0x01 & recv_frames_[0].data[1]) msg.io_fb_lower_beam_headlamp = true;	else msg.io_fb_lower_beam_headlamp = false;
	
						if(0x02 & recv_frames_[0].data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;

						msg.io_fb_turn_lamp = (0xc0 & recv_frames_[0].data[1]) >> 2;

						if(0x10 & recv_frames_[0].data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

						if(0x20 & recv_frames_[0].data[1]) msg.io_fb_clearance_lamp = true;	else msg.io_fb_clearance_lamp = false;

						if(0x40 & recv_frames_[0].data[1]) msg.io_fb_fog_lamp = true;	else msg.io_fb_fog_lamp = false;

						if(0x01 & recv_frames_[0].data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;

						if(0x01 & recv_frames_[0].data[3]) msg.io_fb_fl_impact_sensor = true;	else msg.io_fb_fl_impact_sensor = false;

						if(0x02 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						if(0x04 & recv_frames_[0].data[3]) msg.io_fb_fr_impact_sensor = true;	else msg.io_fb_fr_impact_sensor = false;

						if(0x08 & recv_frames_[0].data[3]) msg.io_fb_rl_impact_sensor = true;	else msg.io_fb_rl_impact_sensor = false;

						if(0x10 & recv_frames_[0].data[3]) msg.io_fb_rm_impact_sensor = true;	else msg.io_fb_rm_impact_sensor = false;

						if(0x20 & recv_frames_[0].data[3]) msg.io_fb_rr_impact_sensor = true;	else msg.io_fb_rr_impact_sensor = false;

						if(0x01 & recv_frames_[0].data[4]) msg.io_fb_fl_drop_sensor = true;	else msg.io_fb_fl_drop_sensor = false;

						if(0x02 & recv_frames_[0].data[4]) msg.io_fb_fm_drop_sensor = true;	else msg.io_fb_fm_drop_sensor = false;

						if(0x04 & recv_frames_[0].data[4]) msg.io_fb_fr_drop_sensor = true;	else msg.io_fb_fr_drop_sensor = false;

						if(0x08 & recv_frames_[0].data[4]) msg.io_fb_rl_drop_sensor = true;	else msg.io_fb_rl_drop_sensor = false;

						if(0x10 & recv_frames_[0].data[4]) msg.io_fb_rm_drop_sensor = true;	else msg.io_fb_rm_drop_sensor = false;

						if(0x20 & recv_frames_[0].data[4]) msg.io_fb_rr_drop_sensor = true;	else msg.io_fb_rr_drop_sensor = false;

						if(0x01 & recv_frames_[0].data[5]) msg.io_fb_estop = true;	else msg.io_fb_estop = false;

						if(0x02 & recv_frames_[0].data[5]) msg.io_fb_joypad_ctrl = true;	else msg.io_fb_joypad_ctrl = false;

						if(0x04 & recv_frames_[0].data[5]) msg.io_fb_charge_state = true;	else msg.io_fb_charge_state = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							io_fb_pub_.publish(msg);
						}

						break;
					}

					
					//bms反馈：电池管理系统
					case 0x98C4E1EF:
					{
						yhs_can_msgs::bms_fb msg;
						msg.bms_fb_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

						msg.bms_fb_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

						msg.bms_fb_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_fb_pub_.publish(msg);
						}

						break;
					}

					//bms_flag反馈:OV是过压保护，当电池电压超过设定值时，BMS会切断电池的充电或放电，以防止电池过充。
								//UV是欠压保护，当电池电压低于设定值时，BMS会切断电池的充电或放电，以防止电池欠压。
								//从接收帧的数据域中提取电池组的SOC、单体电池的过压、欠压、过温、欠温、过流、短路等状态，分别存入msg的相应字段。
					case 0x98C4E2EF:
					{
						yhs_can_msgs::bms_flag_fb msg;
						msg.bms_flag_fb_soc = recv_frames_[0].data[0];

						if(0x01 & recv_frames_[0].data[1]) msg.bms_flag_fb_single_ov = true;	else msg.bms_flag_fb_single_ov = false;

						if(0x02 & recv_frames_[0].data[1]) msg.bms_flag_fb_single_uv = true;	else msg.bms_flag_fb_single_uv = false;

						if(0x04 & recv_frames_[0].data[1]) msg.bms_flag_fb_ov = true;	else msg.bms_flag_fb_ov = false;

						if(0x08 & recv_frames_[0].data[1]) msg.bms_flag_fb_uv = true;	else msg.bms_flag_fb_uv = false;

						if(0x10 & recv_frames_[0].data[1]) msg.bms_flag_fb_charge_ot = true;	else msg.bms_flag_fb_charge_ot = false;

						if(0x20 & recv_frames_[0].data[1]) msg.bms_flag_fb_charge_ut = true;	else msg.bms_flag_fb_charge_ut = false;

						if(0x40 & recv_frames_[0].data[1]) msg.bms_flag_fb_discharge_ot = true;	else msg.bms_flag_fb_discharge_ot = false;

						if(0x80 & recv_frames_[0].data[1]) msg.bms_flag_fb_discharge_ut = true;	else msg.bms_flag_fb_discharge_ut = false;

						if(0x01 & recv_frames_[0].data[2]) msg.bms_flag_fb_charge_oc = true;	else msg.bms_flag_fb_charge_oc = false;

						if(0x02 & recv_frames_[0].data[2]) msg.bms_flag_fb_discharge_oc = true;	else msg.bms_flag_fb_discharge_oc = false;

						if(0x04 & recv_frames_[0].data[2]) msg.bms_flag_fb_short = true;	else msg.bms_flag_fb_short = false;

						if(0x08 & recv_frames_[0].data[2]) msg.bms_flag_fb_ic_error = true;	else msg.bms_flag_fb_ic_error = false;

						if(0x10 & recv_frames_[0].data[2]) msg.bms_flag_fb_lock_mos = true;	else msg.bms_flag_fb_lock_mos = false;

						if(0x20 & recv_frames_[0].data[2]) msg.bms_flag_fb_charge_flag = true;	else msg.bms_flag_fb_charge_flag = false;

						msg.bms_flag_fb_hight_temperature = (float)((short)(recv_frames_[0].data[4] << 4 | recv_frames_[0].data[3] >> 4)) / 10;

						msg.bms_flag_fb_low_temperature = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 8 | recv_frames_[0].data[5])) / 10;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_flag_fb_pub_.publish(msg);
						}

						break;
					}

					default:
						break;
				}

			}

					
		}
	}
}

//数据发送线程
void CanControl::sendData()
{
	ros::Rate loop(100);


	while(ros::ok())
	{

		loop.sleep();
	}

}


void CanControl::run()
{

	ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
	free_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::free_ctrl_cmd>("rear_free_ctrl_cmd", 5, &CanControl::free_ctrl_cmdCallBack, this);

	ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb",5);
	io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb",5);
	r_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::r_wheel_fb>("r_wheel_fb",5);
	l_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::l_wheel_fb>("l_wheel_fb",5);
	bms_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_fb>("bms_fb",5);
	bms_flag_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_fb>("bms_flag_fb",5);

	//打开设备
	// 打开设备，使用socket函数创建一个PF_CAN类型的原始套接字，用于与CAN总线进行通信，如果返回值小于0，说明打开失败，打印错误信息并返回；否则，打印成功信息。
	dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (dev_handler_ < 0) 
	{
		ROS_ERROR(">>open can deivce error!");
		return;
	}
    else
	{
		ROS_INFO(">>open can deivce success!");
	}


	struct ifreq ifr;
	
	std::string can_name("can0");

	strcpy(ifr.ifr_name,can_name.c_str());

	ioctl(dev_handler_,SIOCGIFINDEX, &ifr);


    // bind socket to network interface
	// 绑定套接字到网络接口，使用bind函数将套接字绑定到can0接口，如果返回值小于0，说明绑定失败，打印错误信息并返回。
	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),sizeof(addr));
	if (ret < 0) 
	{
		ROS_ERROR(">>bind dev_handler error!\r\n");
		return;
	}

	//创建接收发送数据线程
	boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));
//	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));

	ros::spin();
	
	close(dev_handler_);
}

}



//主函数
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "yhs_can_control_node");

	yhs_tool::CanControl cancontrol;
	cancontrol.run();

	return 0;
}
