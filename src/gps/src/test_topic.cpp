#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <vector>
#include <string>

void send_message(ros::Publisher& pub, int& counter, const std::vector<std::string>& messages)
{
    std_msgs::String msg;
    
    // 如果计数器超出了信息列表的范围，重新从头开始发布
    int index = counter % messages.size();
    msg.data = messages[index];

    pub.publish(msg);
    counter++;
}

std::vector<std::string> load_messages_from_file(const std::string& file_path)
{
    std::vector<std::string> messages;
    std::ifstream file(file_path);

    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return messages;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // 添加读取的每一行到消息列表
        messages.push_back(line);
    }

    file.close();
    return messages;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("serial_topic", 1000);
    ros::Rate loop_rate(100); // 10 Hz

    // 从文件中加载消息
    // std::vector<std::string> messages = load_messages_from_file("/home/orangepi/ros_ws/src/gps/rtkmsgs/bd1.txt");
    std::vector<std::string> messages = load_messages_from_file("/home/rosubuntu/ros_ws/src/gps/rtkmsgs/bd1.txt");
    if (messages.empty())
    {
        ROS_ERROR("No messages loaded from the file.");
        return 1;
    }

    int counter = 0;
    while (ros::ok())
    {
        send_message(pub, counter, messages);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// BD1:gps->latitude:30.32101833, gps->longitude:120.07105000, gps->qual:4, gps->direction:0.00000000"

// BD2:gps->latitude:30.32098833, gps->longitude:120.07123333, gps->qual:4, gps->direction:0.00000000"

// BD3:gps->latitude:30.32086333, gps->longitude:120.07125000, gps->qual:4, gps->direction:0.00000000

























// void send_message(ros::Publisher& pub, int& counter)
// {
//     std_msgs::String msg;

//     if (counter % 2 == 0) {
//         msg.data = "$GNGGA,101620.00,2140.83682949,N,11055.09963460,E,1,20,0.7,59.7145,M,-13.4263,M,,*67\n";
//     } else {
//         msg.data = "$GPRMC,094403.00,A,4004.73794422,N,11614.18999462,E,0.037,5.5,260815,6.5,W,A*35\n";
//     }

//     pub.publish(msg);
//     counter++;
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "serial_publisher");
//     ros::NodeHandle nh;
//     ros::Publisher pub = nh.advertise<std_msgs::String>("serial_topic", 1000);
//     ros::Rate loop_rate(1); // 1 Hz
//     int counter = 0;
//     while (ros::ok())
//     {
//         send_message(pub, counter);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }
 // 1
 // gps->latitude:21.680427, gps->longitude:110.918648, gps->speed:0.003700, gps->direction:19.200001
 // gps->time: 17:51:05
 // 2
 // gps->latitude:21.680228, gps->longitude:110.918682, gps->speed:0.590150, gps->direction:183.800003
 // gps->time: 17:56:09
//  距离: 0.022314 公里
//  方位角: 170.926928 度


//  第二处参考点
//  gps1.latitude = 21.68007020091791;//随机选取：主教左下角
//  gps1.longitude = 110.91824768309247;// 
// 
//  posi_now.latitude = 21.680078262699684;
//  posi_now.longitude = 110.91905925768252;//地图选点：主教右下角
// 距离: 83.996350M
// 方位角: 89.390943°
