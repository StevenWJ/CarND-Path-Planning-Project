#ifndef MY_SUPPORTS_H
#define MY_SUPPORTS_H

#include <cstdio>
#include <fstream>
#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

class fusionData {
  public:
    unsigned int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
};

void predict_cars_movement(double delta_t, std::vector<fusionData> &sensor_data, const vector<double> &maps_x, const vector<double> &maps_y)  
{
  for (int i=0; i<sensor_data.size(); i++) {
    double pred_x;
    double pred_y;
    pred_x  = sensor_data[i].x + sensor_data[i].vx * delta_t;
    pred_y  = sensor_data[i].y + sensor_data[i].vy * delta_t;
    double pred_theta = atan2(pred_y-sensor_data[i].y, pred_x-sensor_data[i].x); 
    std::vector<double> sd = getFrenet(pred_x, pred_y, pred_theta, maps_x, maps_y);
    sensor_data[i].s  = sd[0];
    sensor_data[i].d  = sd[1];
    sensor_data[i].x  = pred_x;
    sensor_data[i].y  = pred_y;
  }
}

int pickLane(double car_s, double car_d, double look_ahead_dist, double head_safe_dist, double tail_safe_dist, double lane_width, int pref_lane, int num_lane, const std::vector<fusionData> &sensor_data)
{
  int car_lane = int(car_d/lane_width);
  double max_s  = car_s + look_ahead_dist;
  double min_s  = car_s - tail_safe_dist;
  
  double min_cost = std::numeric_limits<double>::infinity();
  double target_lane = car_lane;
  double prefer_d = (pref_lane*lane_width+2.0);

  for (int lane=car_lane-1; lane <=car_lane+1; lane++) { 
    if (lane>=0 && lane<num_lane) {
      double max_d  = double(lane+1)*lane_width;
      double min_d  = double(lane)*lane_width;
      double d_cost = double(abs(lane-pref_lane))*1.0;
      double target_d = double(lane)*lane_width + 2.0;
      double cost_multiplier = 1.0 - (1.0/exp(double(abs(lane-pref_lane)))); 
      for (int i=0; i<sensor_data.size(); i++) {
        if (sensor_data[i].s <= max_s && sensor_data[i].s >= min_s && sensor_data[i].d <= max_d && sensor_data[i].d >= min_d) {
          double delta_s = sensor_data[i].s - car_s;
          double dist = fabs(delta_s);
          if ((delta_s >= 0 && dist <= head_safe_dist) || (delta_s <=0 && dist <= tail_safe_dist)) {
              d_cost += 2000.0 * (1.0-exp(-1.0/dist));
          } else if (delta_s >0) {
              d_cost += 200.0 * (1.0-exp(-1.0/dist));
          } 
        }
      }
      double total_cost = d_cost * (1.0 + cost_multiplier);
      std::cout << " lane: " << lane << " cost: " << total_cost << std::endl;
      if (total_cost < min_cost) {
        target_lane = lane;
        min_cost = total_cost;
      }
    }
  }
  return target_lane;
}



double decideAccel(double step_accel, double car_accel, double max_speed, double car_speed, double safe_dist, double car_s, double lane_width, double car_d, const std::vector<fusionData> &sensor_data)
{
  double new_accel = car_accel;
  int car_lane = int(car_d/lane_width);
  bool found = false;
  for (int i=0; i<sensor_data.size(); i++) {
    int sensor_car_lane = int(sensor_data[i].d/lane_width);
    if (car_lane==sensor_car_lane) {
      if (fabs(sensor_data[i].s - car_s) <= safe_dist && sensor_data[i].s >= car_s) {
        found = true;
        if (car_accel > -step_accel) {
          new_accel = car_accel - step_accel;
        }
        break;
      }
    }
  }
  if (!found) {
    if (car_speed < max_speed) {
      new_accel = step_accel;
    } else {
      new_accel = 0.0;
    }
  }
  return new_accel;
}

double computeNewVelocity(double speed, double accel, double speed_limit, double delta_t)
{
  double new_v = speed+(accel*delta_t);
  if (new_v < 0.0) {
    new_v = 0.0;
  }
  if (new_v > speed_limit) {
    new_v = speed_limit;
  }

  return new_v;
} 

double computeTravelDist(double speed, double accel, double delta_t) 
{
  double travel_dist = (speed*delta_t) + (accel*delta_t*delta_t)/2.0;
  return travel_dist;
}

std::vector<double> transMap2Car(const std::vector<double> &map_wp, const std::vector<double> &car_map_wp, double ref_theda)
{
  double shift_x = map_wp[0] - car_map_wp[0];
  double shift_y = map_wp[1] - car_map_wp[1];
  double car_x = (shift_x*cos(0.0-ref_theda)-shift_y*sin(0.0-ref_theda));
  double car_y = (shift_x*sin(0.0-ref_theda)+shift_y*cos(0.0-ref_theda));
  return {car_x, car_y};
}

std::vector<double> transCar2Map(const std::vector<double> &car_wp, const std::vector<double> &car_map_wp, double ref_theda)
{
  double map_x = (car_wp[0]*cos(ref_theda)-car_wp[1]*sin(ref_theda)) + car_map_wp[0];
  double map_y = (car_wp[0]*sin(ref_theda)+car_wp[1]*cos(ref_theda)) + car_map_wp[1];
  return {map_x, map_y};
}

#endif /* MY_SUPPORTS_H */
