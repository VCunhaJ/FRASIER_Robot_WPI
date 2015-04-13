#include <stdlib.h>
#include <ros/ros.h>
#include <computer_sensors/sensor_types.h>
#include <boost/foreach.hpp>

#define NAME_BUFFER_SIZE 50

SensorChipSubFeature::SensorChipSubFeature(SensorChipFeature& feature, sensors_subfeature const *subfeature):feature_(feature), subfeature_(subfeature){
  name_ = subfeature_->name;

  ROS_INFO("\t\tFound Sub-Feature: %s[%d] = %f", getName().c_str(), subfeature_->type, getValue());
}
std::string SensorChipSubFeature::getName(){
  return name_;
}
double SensorChipSubFeature::getValue(){
  double value;
  sensors_get_value(feature_.chip_.internal_name_, subfeature_->number, &value);
  return value;
}
sensors_subfeature_type SensorChipSubFeature::getType(){
  return subfeature_->type;
}



SensorChipFeature::SensorChipFeature(SensorChip& chip, sensors_feature const *feature):chip_(chip), feature_(feature){
  name_ = feature_->name;
  char* label_c_str = sensors_get_label(chip_.internal_name_, feature_);
  label_ = label_c_str;
  free(label_c_str);

  ROS_INFO("\tFound Feature: %s(%s)[%d]", getLabel().c_str(), getName().c_str(), feature_->type);

  enumerate_subfeatures();
}
void SensorChipFeature::enumerate_subfeatures(){
  sensors_subfeature const *subfeature;
  int number = 0;

  while ((subfeature = sensors_get_all_subfeatures(chip_.internal_name_, feature_, &number)) != NULL) {
    sub_features_.push_back(SensorChipSubFeaturePtr(new SensorChipSubFeature(*this, subfeature)));
  }
}
SensorChipSubFeaturePtr SensorChipFeature::getSubFeatureByType(sensors_subfeature_type type){
  BOOST_FOREACH(SensorChipSubFeaturePtr subfeature, sub_features_){
    if(subfeature->getType()==type)
      return subfeature;
  }
  return SensorChipSubFeaturePtr();
}
std::string SensorChipFeature::getName(){
  return name_;
}
std::string SensorChipFeature::getLabel(){
  return label_;
}
sensors_feature_type SensorChipFeature::getType(){
  return feature_->type;
}



SensorChip::SensorChip(sensors_chip_name const *chip_name):internal_name_(chip_name){
  char name_buffer[NAME_BUFFER_SIZE];
  sensors_snprintf_chip_name(name_buffer, NAME_BUFFER_SIZE, internal_name_);
  name_ = name_buffer;

  ROS_INFO("Found Sensor: %s", getName().c_str());
  enumerate_features();
}
void SensorChip::enumerate_features(){
  features_.clear();

  sensors_feature const *feature;
  int number = 0;

  while ((feature = sensors_get_features(internal_name_, &number)) != NULL) {
    sensors_feature_type type = feature->type;
    switch(type){
    case SENSORS_FEATURE_FAN:
      features_.push_back(SensorChipFeaturePtr(new FanSensor(*this, feature)));
      break;
    case SENSORS_FEATURE_TEMP:
      features_.push_back(SensorChipFeaturePtr(new TempSensor(*this, feature)));
      break;
    default:
      features_.push_back(SensorChipFeaturePtr(new OtherSensor(*this, feature)));
      break;
    }
  }
}
std::string SensorChip::getName(){
  return name_;
}

void FanSensor::publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat){
  SensorChipSubFeaturePtr speed = getSubFeatureByType(SENSORS_SUBFEATURE_FAN_INPUT);
  if(speed){
    double speed_val = speed->getValue();
    if(speed_val==0)
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Fan stalled or not connected");
    else
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "FAN OK (%f RPM)", speed_val);
    stat.add("Fan Speed", speed_val);
  }
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "NO FAN Input!!!");
}

void TempSensor::publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat){
  SensorChipSubFeaturePtr temp = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_INPUT);
  SensorChipSubFeaturePtr max_temp = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_MAX);
  SensorChipSubFeaturePtr temp_crit = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_CRIT);
  SensorChipSubFeaturePtr temp_crit_alarm = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_CRIT_ALARM);
  if(temp){
    double temp_val = temp->getValue();
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "TEMP OK (%f C)", temp_val);
    stat.add("Temperature", temp_val);

    sensor_msgs::Temperature temp_msg;
    temp_msg.variance = 0;
    temp_msg.temperature = temp_val;
    temp_msg.header.frame_id = frame_base+"/"+getLabel();
    temp_msg.header.stamp = ros::Time::now();
    temperaturePublisher.publish(temp_msg);

    if(max_temp)
      stat.add("Max Temperature", max_temp->getValue());
    if(temp_crit)
      stat.add("Temperature Critical", temp_crit->getValue());
    if(temp_crit_alarm)
      stat.add("Temperature Critical Alarm", temp_crit_alarm->getValue());
  }
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "NO TEMP Input!!!");
}

void OtherSensor::publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat){
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "");
    BOOST_FOREACH(SensorChipSubFeaturePtr subfeature, sub_features_){
      stat.add(subfeature->getName(), subfeature->getValue());
    }
}
