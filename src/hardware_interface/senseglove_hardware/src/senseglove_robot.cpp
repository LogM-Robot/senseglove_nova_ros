// Copyright 2020 SenseGlove
#include "senseglove_hardware/joint.h"
#include "senseglove_hardware/senseglove_robot.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
// #include "SenseGlove.h"
#include "HapticGlove.h"
#include "NovaGlove.h"
#include "BasicHandModel.h"
#include "HandPose.h"
#include "DeviceList.h"
#include "SenseCom.h"


namespace senseglove
{
SenseGloveRobot::SenseGloveRobot(
  std::shared_ptr<SGCore::HapticGlove> glove, 
  ::std::vector<Joint> jointList, 
  // urdf::Model urdf, 
  int robotIndex, 
  bool is_right)
  : senseglove_(glove)
  , hand_profile_(SGCore::HandProfile::Default(is_right))
  , hand_model_(SGCore::Kinematics::BasicHandModel::Default(is_right))
  , joint_list_(std::move(jointList))
  // , urdf_(std::move(urdf))
  , name_("senseglove/" + std::to_string(int((robotIndex) / 2)))
  , device_type_(this->senseglove_->GetDeviceType())
  , robot_index_(robotIndex)
  , is_right_(is_right)
  , updated_(false)
{
}

std::string SenseGloveRobot::getName() const
{
  return this->name_;
}

int SenseGloveRobot::getIndex() const
{
  return this->robot_index_;
}

bool SenseGloveRobot::getRight()
{
  return this->senseglove_->IsRight();
}


void SenseGloveRobot::calibrteHandProfile()
{
  if (SGCore::SenseCom::ScanningActive()) //check if the Sense Comm is running. If not, warn the end user.
	{
		if (SGCore::HapticGlove::GetGlove(this->senseglove_))
		{
			std::cout << "Connected to a " << (this->senseglove_->IsRight() ? "right" : "left") << "-handed SenseGlove. Staring calibration" << std::endl;

			/* 
			Our goal is to find the min / max sensor values, which correspond to the user opening their hand and making a fist.
			We can only update this range after parsing sensor data, which happens when accessing sensorData, glovePoses or handPoses.
			In our VR use cases, we pull new hand data each frame, and so this min/max range is updated automatically. 
			In this example, we will update the range twice; once when the hand is 'open', once when it is closed into a fist.
			*/ 		

			// Step 1: Open hand - Calibrates extension
			std::cout << std::endl;
			std::cout << "Step 1: Place your hand on a flat surface, like a table, and spread your thumb and fingers." << std::endl;
			// Once you get the hang of this motion, you can do it without the surface.
			std::cout << "Once your hand is in the right position, press any key to continue" << std::endl; 
			std::cin.get();
			
			// This function updates the calibration range of senseglove_.
			this->senseglove_->UpdateCalibrationRange(); // Instead of this method, you can also use the GetSensorData(), GetGlovePose() or GetHandPose function instead.


			// Step 2: Fist - Calibrates 3-finger flexion
			std::cout << std::endl;
			std::cout << "Step 2: Close your hand into a fist for next 5 seconds. Make sure your fingers aren't wrapped around your thumb." << std::endl;
			std::cout << "Once you are ready, press any key to continue" << std::endl;
			std::cin.get();
      // record start time
      auto start = std::chrono::high_resolution_clock::now();
			// update calibration range for 5 seconds
      while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count() < 5)
      {
        // This function updates the calibration range of senseglove_.
        this->senseglove_->UpdateCalibrationRange(); // Instead of this method, you can also use the GetSensorData(), GetGlovePose() or GetHandPose function instead.
      }
      std::cout << "Step 2 Done." << std::endl;

      // Step 3: Calibrates thumb 
      std::cout << std::endl;
			std::cout << "Step 3: move your thumb to the extreme bounds for next 10 seconds." << std::endl;
			std::cout << "Once you are ready, press any key to continue" << std::endl;
			std::cin.get();
      // record start time
      start = std::chrono::high_resolution_clock::now();
			// update calibration range for 10 seconds
      while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count() < 10)
      {
        // This function updates the calibration range of senseglove_.
        this->senseglove_->UpdateCalibrationRange(); // Instead of this method, you can also use the GetSensorData(), GetGlovePose() or GetHandPose function instead.
      }
      std::cout << "Step 3 Done." << std::endl;

			// At this point, we've collected data while the hand was open, and when it was closed. 
			// The calibration range should now have the two extremes to interpolate between.
			// Let's check & ouput the ranges:
			std::vector<SGCore::Kinematics::Vect3D> minRanges, maxRanges;
			this->senseglove_->GetCalibrationRange(minRanges, maxRanges);
			
			// The calibration ranges contain the x, y, z values, which represent the pronation/supination, flexion/extension, and abduction/adduction movements respectively, in radians. 
			// For readability's sake, we'll print out the flexion/extension values in degrees.
			float rad2Deg = 180 / M_PI;
			std::cout << std::endl << "Evaluated the following calibration range for extension/flexion" << std::endl;
			std::cout << "Extensions: ";
			for (int f = 0; f < minRanges.size(); f++)
			{
				std::cout << std::to_string((int)(rad2Deg * minRanges[f].y));
				if (f < minRanges.size() - 1) { std::cout << ", "; }
			}
			std::cout << std::endl << "Flexions: ";
			for (int f = 0; f < maxRanges.size(); f++)
			{
				std::cout << std::to_string((int)(rad2Deg * maxRanges[f].y));
				if (f < maxRanges.size() - 1) { std::cout << ", "; }
			}
			std::cout << std::endl;

			// Now we apply the calibration to the default profile
			this->senseglove_->ApplyCalibration(hand_profile_);

			// And can now use it to calculate handPoses
			SGCore::HandPose handPose;
			if (this->senseglove_->GetHandPose(hand_profile_, handPose))
			{
				std::cout << std::endl << "With these ranges, we've calculated the following hand angles:" << std::endl;
				std::cout << handPose.ToString() << std::endl;
			}
			else
			{
				std::cout << "Something went wrong while trying to calucate a handPose. Perhaps a packet was dropped or an exception occurs." << std::endl;
			}
			
			// // Finally, we can store the profile in its serialized form to use later
			// std::string serializedProfile = hand_profile_.Serialize();

			// // And Deserialize it back into a useable profile
			// SGCore::HandProfile loadedProfile = SGCore::HandProfile::Deserialize(serializedProfile);
		}
	}
	else
		std::cout << "SenseComm is not running. Please start it and try again." << std::endl;

	std::cout << "=======================================" << std::endl;
	std::cout << "Press any key to exit calibration." << std::endl;
	std::cin.get();
}


void SenseGloveRobot::updateTrackerData(SGCore::Kinematics::Vect3D position, SGCore::Kinematics::Quat rotation)
{
  this->trackerPosition = position;
  this->trackerRotation = rotation;
}


Joint& SenseGloveRobot::getJoint(::std::string jointName)
{
  for (auto& joint : joint_list_)
  {
    if (joint.getName() == jointName)
    {
      return joint;
    }
  }
  throw std::out_of_range("Could not find joint with name " + jointName);
}

Joint& SenseGloveRobot::getJoint(size_t index)
{
  return this->joint_list_.at(index);
}

// SGCore::Kinematics::Vect3D SenseGloveRobot::getHandPos(int i)
// {
//   // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
//   // SG uses vector of vectors and ROS uses one long array
//   return hand_pose_.jointPositions[std::floor(i / 4)][i % 4];
// }

SGCore::Kinematics::Vect3D SenseGloveRobot::getWristPos()
{return wrist_position_;}

SGCore::Kinematics::Quat SenseGloveRobot::getWristRot()
{return wrist_rotation_;}

SGCore::Kinematics::Vect3D SenseGloveRobot::getHandAngles(int i)
{
  return hand_pose_.handAngles[std::floor(i / 3)][i % 3];
}

SGCore::Kinematics::Vect3D SenseGloveRobot::getJointPos(int i)
{
  // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
  // SG uses vector of vectors and ROS uses one long array
  return hand_pose_.jointPositions[std::floor(i / 4)][i % 4];
}

SGCore::Kinematics::Quat SenseGloveRobot::getJointRot(int i)
{
  return hand_pose_.jointRotations[std::floor(i / 4)][i % 4];
}


// SGCore::Kinematics::Vect3D SenseGloveRobot::getFingerTip(int i)
// {
//   // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
//   // SG uses vector of vectors and ROS uses one long array
//   return glove_pose_.CalculateFingerTips(hand_profile_)[i];
// }

void SenseGloveRobot::actuateEffort(std::vector<double> effort_command)
{
  // if (SGCore::DeviceList::SenseComRunning())  // check if the Sense Comm is running. If not, warn the end user.
  if (SGCore::SenseCom::ScanningActive())
  {
    std::vector<int> int_effort(effort_command.begin(), effort_command.end());
    if (effort_command[0] + effort_command[1] + effort_command[2] + effort_command[3] + effort_command[4] < 10.0)  // less than noticable ffb
    {
      this->senseglove_->SendHaptics(SGCore::Haptics::SG_FFBCmd(SGCore::Haptics::SG_FFBCmd::off));
    }
    else
    {
      this->senseglove_->SendHaptics(
          SGCore::Haptics::SG_FFBCmd(int_effort[0], int_effort[1], int_effort[2], int_effort[3], int_effort[4]));
    }
  }
}

void SenseGloveRobot::actuateEffort(double e_0, double e_1, double e_2, double e_3, double e_4)
{
  std::vector<double> efforts = { e_0, e_1, e_2, e_3, e_4 };
  this->actuateEffort(efforts);
}

void SenseGloveRobot::actuateBuzz(std::vector<double> buzz_command)
{
  std::vector<int> int_buzz(buzz_command.begin(), buzz_command.end());
  if (buzz_command[0] + buzz_command[1] + buzz_command[2] + buzz_command[3] + buzz_command[4] < 10.0)  // less than noticable buzz
  {
    this->senseglove_->SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Haptics::SG_BuzzCmd::off));
  }
  else
  {
    this->senseglove_->SendHaptics(
        SGCore::Haptics::SG_BuzzCmd(int_buzz[0], int_buzz[1], int_buzz[2], int_buzz[3], int_buzz[4]));
  }
}

void SenseGloveRobot::actuateBuzz(double b_0, double b_1, double b_2, double b_3, double b_4)
{
  std::vector<double> buzzes = { b_0, b_1, b_2, b_3, b_4 };
  this->actuateBuzz(buzzes);
}

void SenseGloveRobot::actuateEffortBuzz(std::vector<double> effort_command, std::vector<double> buzz_command)
{
  SGCore::Haptics::SG_FFBCmd* FF_cmd;
  SGCore::Haptics::SG_BuzzCmd* buzz_cmd;
  if (SGCore::SenseCom::ScanningActive())
  {
    std::vector<int> int_effort(effort_command.begin(), effort_command.end());
    if (effort_command[0] + effort_command[1] + effort_command[2] + effort_command[3] + effort_command[4] < 10.0)  // less than noticable ffb
    {

      FF_cmd = new SGCore::Haptics::SG_FFBCmd(SGCore::Haptics::SG_FFBCmd::off);
    }
    else
    {
      FF_cmd = new SGCore::Haptics::SG_FFBCmd(int_effort[0], int_effort[1], int_effort[2], int_effort[3], int_effort[4]);
    }

    std::vector<int> int_buzz(buzz_command.begin(), buzz_command.end());
    if (buzz_command[0] + buzz_command[1] + buzz_command[2] + buzz_command[3] + buzz_command[4] < 10.0)  // less than noticable buzz
    {
      buzz_cmd = new SGCore::Haptics::SG_BuzzCmd(SGCore::Haptics::SG_BuzzCmd::off);
    }
    else
    {
      buzz_cmd = new SGCore::Haptics::SG_BuzzCmd(int_buzz[0], int_buzz[1], int_buzz[2], int_buzz[3], int_buzz[4]);
    } 
    this->senseglove_->SendHaptics(*FF_cmd, *buzz_cmd);
  }
}

void SenseGloveRobot::stopActuating()
{
  this->senseglove_->StopHaptics();
}

size_t SenseGloveRobot::size() const
{
  return this->joint_list_.size();
}

SenseGloveRobot::iterator SenseGloveRobot::begin()
{
  return this->joint_list_.begin();
}

SenseGloveRobot::iterator SenseGloveRobot::end()
{
  return this->joint_list_.end();
}

SenseGloveRobot::~SenseGloveRobot()
{
}

bool SenseGloveRobot::updateGloveData(const ros::Duration period)
{
  // bool glove_update = false;
  bool wrist_update = false;
  bool hand_update = false;
  // if (senseglove_->GetSensorData(sensor_data_))  // if GetSensorData is true, we have sucesfully recieved data
  // {
  //   // ROS_DEBUG("successfully update glove sensor data");
  //   for (auto& joint : joint_list_)
  //   {
  //     joint.position_ = sensor_data_.sensorAngles[joint.joint_index_ / 4][joint.joint_index_ % 4];
  //     double intermediate_vel = (sensor_data_.sensorAngles[joint.joint_index_ / 4][joint.joint_index_ % 4] - joint.velocity_);
  //     if (intermediate_vel != 0.0 and period.toSec() != 0.0)
  //     {
  //       joint.velocity_ = intermediate_vel / 1;
  //     }
  //     else
  //     {
  //       joint.velocity_ = 0.0;
  //     }
  //   }
  // }
  // if (!senseglove_->GetGlovePose(glove_pose_))
  // {
  //   ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated glove pose data");
  // }
  // else
  // {
  //   this->tip_positions_ = this->glove_pose_.CalculateFingerTips(this->hand_profile_);
  //   glove_update = true;
  // }

  // Get wrist location
  this->senseglove_->GetWristLocation(this->trackerPosition, this->trackerRotation, SGCore::PosTrackingHardware::ViveTracker, this->wrist_position_, this->wrist_rotation_);
  // this->novaglove_->CalculateWristLocation(this->trackerPosition, this->trackerRotation, SGCore::PosTrackingHardware::ViveTracker, true, this->wrist_position_, this->wrist_rotation_);

  // SGCore::Kinematics::Vect3D L_pos_offset;
  // SGCore::Kinematics::Quat L_rot_offset;
  // SGCore::Kinematics::Vect3D R_pos_offset;
  // SGCore::Kinematics::Quat R_rot_offset;
  // this->tracking_.GetNovaOffset_Tracker_Glove(SGCore::PosTrackingHardware::ViveTracker, false, L_pos_offset, L_rot_offset);
  // this->tracking_.GetNovaOffset_Tracker_Glove(SGCore::PosTrackingHardware::ViveTracker, true, R_pos_offset, R_rot_offset);
  // if (this->is_right_)
  // {
  //   this->tracking_.CalculateLocation(this->trackerPosition, this->trackerRotation, R_pos_offset, R_rot_offset, this->wrist_position_, this->wrist_rotation_);
  // }
  // else
  // {
  //   this->tracking_.CalculateLocation(this->trackerPosition, this->trackerRotation, L_pos_offset, L_rot_offset, this->wrist_position_, this->wrist_rotation_);
  // }

  wrist_update = true;

  if (!this->senseglove_->GetHandPose(this->hand_model_, this->hand_profile_, this->hand_pose_))
  {
    ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated hand pose data");
  }
  else
  {
    hand_update = true;
  }

  updated_ |= (wrist_update and hand_update);
  return updated_;
}

// const urdf::Model& SenseGloveRobot::getUrdf() const
// {
//   return this->urdf_;
// }

}  // namespace senseglove
