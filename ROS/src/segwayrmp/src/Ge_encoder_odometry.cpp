//
// Created by efan on 3/19/20.
//

#include "Ge_encoder_odometry.h"

#include <cmath>

Ge_encoder_odometry::Ge_encoder_odometry() { init(); }

void Ge_encoder_odometry::estimate(SensorData *rawdata) {
  m_num_out++;
  int lefttick = rawdata->ticks.leftTicks;
  int righttick = rawdata->ticks.rightTicks;
  int64_t ts_ticks = rawdata->ticks.TimeStamp;
  double omegaImu = rawdata->baseImu.baseYaw;

  m_current_ts = ts_ticks;
  if (!m_isInit) return;

  m_current_t_read = ts_ticks;
  int64_t interval = m_current_t_read - m_prev_t_read;
  if (m_initial) {
    m_initial = false;
    if (m_if_record) {
      if (m_record_file.is_open()) {
        m_record_file << std::to_string(0) << " "
                      << std::to_string(m_current_ts) << " "
                      << std::to_string(m_odometry_.pose_.x) << " "
                      << std::to_string(m_odometry_.pose_.y) << " "
                      << std::to_string(m_odometry_.pose_.orientation) << " "
                      << std::to_string(omegaImu) << " "
                      << std::to_string(m_omegaImu_bias) << " "
                      << std::to_string(omegaImu - m_omegaImu_bias) << " "
                      << std::to_string(0) << std::endl;
      }
    }
    m_prev_left_count = lefttick;
    m_prev_right_count = righttick;
    m_prev_t_read = m_current_t_read;
    m_odometry_.TimeStamp = ts_ticks;
    m_odometry_.twist_.v_x = 0;
    m_odometry_.twist_.v_y = 0;
    m_odometry_.twist_.w_z = 0;
    return;
  }
  int64_t delta_left_count = lefttick - m_prev_left_count;
  int64_t delta_right_count = righttick - m_prev_right_count;

  if (delta_left_count == 0 && delta_right_count == 0) {
    m_num_static++;
    if (m_num_static > m_num_ignore_static) {
      m_imu_static_sum += omegaImu;
      m_prev_t_read = m_current_t_read;
      m_odometry_.TimeStamp = ts_ticks;
      m_odometry_.twist_.v_x = 0;
      m_odometry_.twist_.v_y = 0;
      m_odometry_.twist_.w_z = 0;
    } else {
      m_imu_static_sum = 0;
    }
    if (m_if_record) {
      if (m_record_file.is_open()) {
        m_record_file << std::to_string(interval) << " "
                      << std::to_string(m_current_ts) << " "
                      << std::to_string(m_odometry_.pose_.x) << " "
                      << std::to_string(m_odometry_.pose_.y) << " "
                      << std::to_string(m_odometry_.pose_.orientation) << " "
                      << std::to_string(omegaImu) << " "
                      << std::to_string(m_omegaImu_bias) << " "
                      << std::to_string(omegaImu - m_omegaImu_bias) << " "
                      << std::to_string(0) << std::endl;
      }
    }
    return;
  } else {
    m_num_static = 0;
    m_imu_static_sum = 0;
  }
  if (m_num_static > 50) {
    m_omegaImu_bias = m_imu_static_sum / m_num_static;
  }
  /*delta_left_count = delta_left_count < -2147483648 ? delta_left_count +=
  4294967296 : delta_left_count; delta_right_count = delta_right_count <
  -2147483648 ? delta_right_count += 4294967296 : delta_right_count;
  delta_left_count = delta_left_count > 2147483648 ? delta_left_count -=
  4294967296 : delta_left_count; delta_right_count = delta_right_count >
  2147483648 ? delta_right_count -= 4294967296 : delta_right_count;*/
  double delta_theta = delta_right_count * m_count_to_right_rot_ -
                       delta_left_count * m_count_to_left_rot_;
  double left_distance = delta_left_count * m_distance_per_count_;
  double right_distance = delta_right_count * m_distance_per_count_;
  double duration = static_cast<double>(interval);
  double avg_distance = (left_distance + right_distance) / 2;
  double omega_tick = delta_theta / duration * 1000000;
  bool is_slip = false;
  double omega_estimate;
  omega_estimate = omegaImu - m_omegaImu_bias;
  delta_theta = omega_estimate * duration / 1000000;
#if 1
  if ((((omega_tick >= 0 && omegaImu <= 0) ||
        (omega_tick <= 0 && omegaImu >= 0)) &&
       abs(omega_tick - omegaImu) > 0.2) ||
      (((omega_tick > 0 && omegaImu > 0) || (omega_tick < 0 && omegaImu < 0)) &&
       (abs(omega_tick - omegaImu) >
        std::min((abs(omegaImu) > 0.05 ? (3 * abs(omegaImu)) : 0.3),
                 std::max(0.2, 3 * omega_tick))))) {
    is_slip = true;
  }
  if (is_slip) {
    avg_distance = abs(left_distance) < abs(right_distance) ? left_distance
                                                            : right_distance;
  }
#endif
  double delta_x, delta_y;
  delta_x =
      avg_distance * std::cos(m_odometry_.pose_.orientation + delta_theta / 2);
  delta_y =
      avg_distance * std::sin(m_odometry_.pose_.orientation + delta_theta / 2);
  m_odometry_.pose_.x += delta_x;
  m_odometry_.pose_.y += delta_y;
  m_odometry_.pose_.orientation += delta_theta;
  m_odometry_.pose_.orientation = m_odometry_.pose_.orientation > M_PI
                                      ? m_odometry_.pose_.orientation - 2 * M_PI
                                      : m_odometry_.pose_.orientation;
  m_odometry_.pose_.orientation = m_odometry_.pose_.orientation < -M_PI
                                      ? m_odometry_.pose_.orientation + 2 * M_PI
                                      : m_odometry_.pose_.orientation;
  m_odometry_.TimeStamp = ts_ticks;
  double linear_speed = avg_distance / duration * 1000000;
  m_odometry_.twist_.v_x =
      linear_speed * std::cos(m_odometry_.pose_.orientation);
  m_odometry_.twist_.v_y =
      linear_speed * std::sin(m_odometry_.pose_.orientation);
  m_odometry_.twist_.w_z = omega_estimate;
  m_prev_left_count = lefttick;
  m_prev_right_count = righttick;
  if (m_current_t_read - m_prev_t_read > 500000) {
    ROS_WARN_STREAM("gap is too big, current_time("
                    << m_current_t_read << "), prev_time(" << m_prev_t_read);
  }
  m_prev_t_read = m_current_t_read;
  if (m_if_record) {
    if (m_record_file.is_open()) {
      m_record_file << std::to_string(interval) << " "
                    << std::to_string(m_current_ts) << " "
                    << std::to_string(m_odometry_.pose_.x) << " "
                    << std::to_string(m_odometry_.pose_.y) << " "
                    << std::to_string(m_odometry_.pose_.orientation) << " "
                    << std::to_string(omegaImu) << " "
                    << std::to_string(m_omegaImu_bias) << " "
                    << std::to_string(omega_estimate) << " "
                    << std::to_string(delta_theta) << std::endl;
    } else {
      ROS_WARN("the Record file is not open!");
    }
  }
}

void Ge_encoder_odometry::init() {
  m_isInit = true;
  m_wheels_distance = 0.392;
  m_distance_per_count_ = 0.202 * M_PI / 4096;
  m_count_to_right_rot_ = m_distance_per_count_ / m_wheels_distance;
  m_count_to_left_rot_ = m_distance_per_count_ / m_wheels_distance;
  ros::NodeHandle pnh = ros::NodeHandle("record");
  pnh.param<bool>("if_record", m_if_record, false);
  pnh.param<std::string>("record_file_path", m_record_file_path,
                         "/home/linaro/catkin_ws/");
  if (m_if_record) {
    std::string cmd_str_mk;
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char dt[100];
    sprintf(dt, "%04d-%02d-%02d_%02d-%02d-%02d", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    std::string slipTest_data_record_dir =
        m_record_file_path + std::string(dt) + "/";
    cmd_str_mk = "mkdir -p \"" + slipTest_data_record_dir + "\"";
    std::string file_path = slipTest_data_record_dir + "pose.txt";
    ROS_INFO_STREAM("file_path: " << file_path);
    m_record_file.open(file_path.data(), std::ofstream::out);
  }
}

bool Ge_encoder_odometry::add_imubase(SensorData::BaseImu &imudata) {
  bool res_flag = false;
  m_imu_record_.imu_buffer_.push_back(imudata);
  if (m_imu_record_.first) {
    m_imu_record_.first = false;
    return res_flag;
  }
  if (!m_futureTicks_.empty() &&
      imudata.TimeStamp >= m_futureTicks_.front().TimeStamp) {
    res_flag = add_ticks((m_futureTicks_.front()));
    m_futureTicks_.pop();
  }
  return res_flag;
}

bool Ge_encoder_odometry::add_ticks(SensorData::Ticks &ticksdata) {
  bool flag = false;
  if (m_imu_record_.imu_buffer_.empty() ||
      ticksdata.TimeStamp < m_imu_record_.imu_buffer_.front().TimeStamp) {
    ROS_WARN_STREAM("" << ticksdata.TimeStamp
                       << ": the ticks is elear than imu");
  } else if (ticksdata.TimeStamp <=
             m_imu_record_.imu_buffer_.back().TimeStamp) {
    SensorData::BaseImu base_imu =
        m_imu_record_.getAverageImu(ticksdata.TimeStamp);
    estimate(created(base_imu, ticksdata));
    flag = true;
  } else if (ticksdata.TimeStamp > m_imu_record_.imu_buffer_.back().TimeStamp) {
    m_futureTicks_.push(ticksdata);
  }
  return flag;
}

SensorData *Ge_encoder_odometry::created(SensorData::BaseImu &imudata,
                                         SensorData::Ticks &ticksdata) {
  SensorData *new_sensordata = new SensorData();
  new_sensordata->baseImu = imudata;
  new_sensordata->ticks = ticksdata;
  return new_sensordata;
}

Odometry Ge_encoder_odometry::GetOdometry() { return m_odometry_; }
