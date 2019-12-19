#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "supports.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          int prev_path_size = previous_path_x.size();

          vector<double> next_x_vals;
          vector<double> next_y_vals;

         /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          double      LANE_WIDTH=4.0;
          int         NUM_LANES=3;
          double      CYCLE_TIME=0.02; // frame rate is 20 ms
          int         NUM_WAYP=50;
          double      SPEED_LIMIT=0.98*50.0*1609.34/3600.0;
          double      SPEED_ACC_STEP=8.0;
          int         PREF_LANE=1;
          double      LOOK_AHEAD_DIST=100.0;
          double      HEAD_SAFE_DIST=20.0;
          double      TAIL_SAFE_DIST=10.0;
          double      curr_spd_acc=0.0;
          double      max_s = 6945.554;
          double      LATENCY = 0.3; // expected network latency
          double      curr_yaw=car_yaw;
          double      curr_s=car_s;
          double      curr_d=car_d;
          double      curr_x=car_x;
          double      curr_y=car_y;
          double      curr_v=car_speed;
          double      target_d=PREF_LANE*4.0+2.0;
        

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;
          double ref_theta  = deg2rad(ref_yaw);
          double ref_x_prev = ref_x;
          double ref_y_prev = ref_y;
          double ref_s  = car_s;
          double ref_d  = car_d;

          int num_wp_kept = int(ceil(LATENCY/CYCLE_TIME));

          int last_idx = (num_wp_kept < prev_path_size) ? num_wp_kept : prev_path_size;

          if (last_idx < 2 ) {
            ref_x_prev = ref_x - cos(ref_theta);
            ref_y_prev = ref_y - sin(ref_theta);
            curr_v  = car_speed;
            vector<double> next_car_wp = transMap2Car({ref_x_prev, ref_y_prev}, {ref_x, ref_y}, ref_theta);
            ptsx.push_back(next_car_wp[0]);
            ptsy.push_back(next_car_wp[1]);
            next_car_wp = transMap2Car({ref_x, ref_y}, {ref_x, ref_y}, ref_theta);
            ptsx.push_back(next_car_wp[0]);
            ptsy.push_back(next_car_wp[1]);
          } else {
            ref_x = previous_path_x[last_idx-1];
            ref_y = previous_path_y[last_idx-1];
            ref_x_prev = previous_path_x[last_idx-2];
            ref_y_prev = previous_path_y[last_idx-2];
            ref_theta  = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            ref_yaw    = rad2deg(ref_theta);
            curr_v  = sqrt(pow(ref_x-ref_x_prev, 2.0)+pow(ref_y-ref_y_prev, 2.0))/CYCLE_TIME;
            vector<double> sd = getFrenet(ref_x, ref_y, ref_theta, map_waypoints_x, map_waypoints_y); 
            ref_s  = sd[0];
            ref_d  = sd[1];
          }

          if (last_idx >=3) {
            double ref_x_prev2 = previous_path_x[last_idx-3];
            double ref_y_prev2 = previous_path_y[last_idx-3];
            double prev_v  = sqrt(pow(ref_y_prev-ref_y_prev2, 2.0)+pow(ref_x_prev-ref_x_prev2, 2.0))/CYCLE_TIME;
            curr_spd_acc = curr_v - prev_v;             
          }

          vector<fusionData> cars_sensor_data; 
          for (int i=0; i<sensor_fusion.size(); i++) {
            fusionData  car_sensed;
            car_sensed.id = sensor_fusion[i][0];
            car_sensed.x  = sensor_fusion[i][1];
            car_sensed.y  = sensor_fusion[i][2];
            car_sensed.vx = sensor_fusion[i][3];
            car_sensed.vy = sensor_fusion[i][4];
            car_sensed.s  = sensor_fusion[i][5];
            car_sensed.d  = sensor_fusion[i][6];
            cars_sensor_data.push_back(car_sensed);
          }

          double prev_wp_x = -std::numeric_limits<double>::infinity();
          for (int i=0; i<last_idx; i++) {
            vector<double> next_car_wp = transMap2Car({previous_path_x[i], previous_path_y[i]}, {ref_x, ref_y}, ref_theta);
            if (next_car_wp[0] > prev_wp_x) {
              ptsx.push_back(next_car_wp[0]);
              ptsy.push_back(next_car_wp[1]);
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
              predict_cars_movement(CYCLE_TIME, cars_sensor_data, map_waypoints_x, map_waypoints_y);
              prev_wp_x = next_car_wp[0];
            }
          }

          int target_lane = pickLane(ref_s, ref_d, LOOK_AHEAD_DIST, HEAD_SAFE_DIST, TAIL_SAFE_DIST, LANE_WIDTH, PREF_LANE, 3, cars_sensor_data);

          int s_steps = 3;
          double dest_d = double(target_lane)*4.0 + 2.0;
          int ref_lane = int(ref_d/LANE_WIDTH);
          int lane_delta = abs(target_lane-ref_lane);
          double d_pitch = (dest_d - ref_d)/double(s_steps*2);
          double plan_s = ref_s;
          double plan_d = ref_d + (dest_d - ref_d)/2.0;
          double s_pitch = 30.0;
          for (int j=0; j<s_steps; j++) {
            plan_s += s_pitch;
            plan_d += d_pitch;
            vector<double> next_map_wp = getXY(plan_s, plan_d, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
            vector<double> next_car_wp = transMap2Car(next_map_wp, {ref_x, ref_y}, ref_theta);
            ptsx.push_back(next_car_wp[0]);
            ptsy.push_back(next_car_wp[1]);
          }

          tk::spline spl; 
          spl.set_points(ptsx, ptsy);
          
          double x_p  = 0.0;
          double y_p  = 0.0;
          double prev_theta = 0.0;
          double prev_x_p = 0.0;
          double prev_y_p = 0.0;
          double prev_yaw = 0.0;
          double plan_cost = 0.0;
          double plan_v = curr_v;
          double plan_acc = curr_spd_acc;
          double prev_plan_acc = curr_spd_acc;
          double travel_dist = 0.0;
          double brake_cnt = 0.0;
          double dist_corr = 0.0;
          vector<double> prev_map_wp = transCar2Map({x_p, y_p}, {ref_x, ref_y}, ref_theta);
          for (int i=0; i < NUM_WAYP - last_idx; i++) {
            double d_dist = computeTravelDist(plan_v, plan_acc, CYCLE_TIME);
            plan_v  = computeNewVelocity(plan_v, plan_acc, SPEED_LIMIT, CYCLE_TIME);

            if (dist_corr < 0.0) {
              d_dist += dist_corr;
            }
            x_p += d_dist*cos(prev_theta);
            double y_p = spl(x_p);
            travel_dist += d_dist;              
            double real_dist = sqrt(pow(x_p-prev_x_p, 2.0)+pow(y_p-prev_y_p, 2.0));
            dist_corr = d_dist - real_dist; 

            vector<double> map_wp = transCar2Map({x_p, y_p}, {ref_x, ref_y}, ref_theta);

            next_x_vals.push_back(map_wp[0]);
            next_y_vals.push_back(map_wp[1]);

            prev_theta  = atan2(y_p-prev_y_p, x_p-prev_x_p);
            prev_yaw    = rad2deg(prev_theta);
            prev_x_p    = x_p;
            prev_y_p    = y_p;

            double prev_map_theta  = atan2(map_wp[1]-prev_map_wp[1], map_wp[0]-prev_map_wp[0]);

            predict_cars_movement(CYCLE_TIME, cars_sensor_data, map_waypoints_x, map_waypoints_y);
            vector<double> map_sd = getFrenet(map_wp[0], map_wp[1], prev_map_theta, map_waypoints_x, map_waypoints_y); 

            prev_plan_acc = plan_acc;
            plan_acc = decideAccel(SPEED_ACC_STEP, prev_plan_acc, SPEED_LIMIT, plan_v, HEAD_SAFE_DIST+plan_v, map_sd[0], LANE_WIDTH, map_sd[1], cars_sensor_data);
            prev_map_wp = map_wp;
          }
 

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
