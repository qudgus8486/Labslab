/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>
serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    int k=1;
    ros::Rate loop_rate(10);
    while(ros::ok()){

        uint8_t a;
        a++;
        ros::spinOnce();
        //2000 07D0
        //-2000 F830

        for(int i=0; i<10;i++) if(i==10) k=0;
        
        uint8_t ask[14]={0x53,0x54,0x58,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x01,a,0x0D,0x0A};
        uint8_t answer[18]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//        uint8_t ask[14]={0x0A,0x0D,a,0x01,0xDD,0x00,0x00,0x00,0x00,0x00,0x01,0x58,0x54,0x53};
    
        //for(int i=0;i<13;i++)         printf("%x ",ask[i]);
        //        printf("\n");

        
        ser.write(ask,14);//S  0x53
        std::string anwer="/n";
        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;

            ser.read(answer,18);
            if(answer[16]==0x0D && answer[17]==0x0A)
            {   for(int i=0;i<18;i++)         printf("%x ",answer[i]);
                printf("\n");
}
            read_pub.publish(result);
            
        }
        loop_rate.sleep();

    }
}

