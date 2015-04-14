/*
 * CPUTempFinder.cpp
 *
 *  Created on: Feb 14, 2012
 *      Author: oryx
 */

#include <sensors/sensors.h>
#include <string>
#include <sensor_msgs/Temperature.h>
#include <computer_sensors/sensor_types.h>
#include <ros/ros.h>
#include <vector>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/foreach.hpp>
#include <device_driver_base/driver_util.h>

std::vector<SensorChipPtr> sensor_chips_;

void enumerate_sensors(){
  sensor_chips_.clear();
  
  sensors_chip_name const *chip_name;
  int number = 0;
  while ((chip_name = sensors_get_detected_chips(NULL, &number)) != NULL){
    sensor_chips_.push_back(SensorChipPtr(new SensorChip(chip_name)));
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sensor_monitor");
  ros::NodeHandle nh;

  define_and_get_param(double, diagnostic_period, "~diagnostic_period", 2);//updater retrieves this value
  define_and_get_param(std::string, temperature_topic, "~temperature_topic", "temperature");
  define_and_get_param(std::string, computer_frame, "~computer_frame", "none");


  ros::Publisher temperaturePublisher = nh.advertise<sensor_msgs::Temperature>(temperature_topic, 100);

  sensors_cleanup();
  sensors_init(NULL);

  enumerate_sensors();

  diagnostic_updater::Updater updater;
  updater.setHardwareID(computer_frame);


  BOOST_FOREACH(SensorChipPtr sensor_chip, sensor_chips_){
    BOOST_FOREACH(ComputerSensorPtr sensor, sensor_chip->features_){
      updater.add(sensor->getDiagnosticName(), boost::bind(&ComputerSensor::publishValues, sensor, computer_frame, temperaturePublisher, _1));
    }
  }


  ros::Rate rate(diagnostic_period*2);
  while(ros::ok()){
    updater.update();
    ros::spinOnce();
    rate.sleep();
  }

  sensors_cleanup();
}

