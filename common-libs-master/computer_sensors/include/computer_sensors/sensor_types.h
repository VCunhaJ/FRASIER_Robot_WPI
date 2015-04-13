#ifndef SENSOR_TYPES_H_
#define SENSOR_TYPES_H

#include <stdlib.h>
#include <sensors/sensors.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/Temperature.h>

class SensorChipSubFeature;
class SensorChipFeature;
class SensorChip;
class ComputerSensor;

typedef boost::shared_ptr<SensorChip> SensorChipPtr;
typedef boost::shared_ptr<SensorChipFeature> SensorChipFeaturePtr;
typedef boost::shared_ptr<SensorChipSubFeature> SensorChipSubFeaturePtr;
typedef boost::shared_ptr<ComputerSensor> ComputerSensorPtr;


class ComputerSensor {
 public:
  virtual std::string getDiagnosticName() = 0;
  virtual void publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat) = 0;
};


class SensorChipSubFeature{
private:
  std::string name_;
  SensorChipFeature& feature_;
  sensors_subfeature const *subfeature_;
public:
  SensorChipSubFeature(SensorChipFeature& feature, sensors_subfeature const *subfeature);
  std::string getName();
  sensors_subfeature_type getType();
  double getValue();
};

class SensorChipFeature : public ComputerSensor{
private:
  std::string name_;
  std::string label_;
  SensorChip& chip_;
  sensors_feature const *feature_;
  void enumerate_subfeatures();
public:
  std::vector<SensorChipSubFeaturePtr> sub_features_;
  SensorChipFeature(SensorChip& chip, sensors_feature const *feature);
  std::string getName();
  std::string getLabel();
  sensors_feature_type getType();
  SensorChipSubFeaturePtr getSubFeatureByType(sensors_subfeature_type type);

  friend class SensorChipSubFeature;
};

class SensorChip{
private:
  std::string name_;
  sensors_chip_name const *internal_name_;
  void enumerate_features();
public:
  std::vector<SensorChipFeaturePtr> features_;
  SensorChip(sensors_chip_name const *chip_name);
  std::string getName();

  friend class SensorChipFeature;
  friend class SensorChipSubFeature;
};


class FanSensor : public SensorChipFeature{
 public:
  FanSensor(SensorChip& chip, sensors_feature const *feature):SensorChipFeature(chip, feature){}
  virtual std::string getDiagnosticName(){return getLabel();}
  virtual void publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat);
};

class TempSensor : public SensorChipFeature{
 public:
  TempSensor(SensorChip& chip, sensors_feature const *feature):SensorChipFeature(chip, feature){}
  virtual std::string getDiagnosticName(){return getLabel();}
  virtual void publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat);
};

class OtherSensor : public SensorChipFeature{
 public:
  OtherSensor(SensorChip& chip, sensors_feature const *feature):SensorChipFeature(chip, feature){}
  virtual std::string getDiagnosticName(){return getLabel();}
  virtual void publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat);
};


#endif
