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
          
          // Start in the center lane. Left = 0, center = 1, right = 2
          int lane = 1;

          // Set the standard speed to hit in mph
          double target_vel = 49.5;

          

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Used to determine how many more points we need to generate for a full path
          int prev_path_size = previous_path_x.size();
          
          
          // Widely spaced points that will be used to generate a spline
          vector<double> spline_pts_x;
          vector<double> spline_pts_y;
          
          // Get the current state of the car
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // If the path was almost finished by the last cycle then just make a new path from scratch starting at the car's location
          if(prev_path_size < 2)
          {
            // Extrapolate back to generate a point tangent to the car's current position
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            // Push the previous and current car location onto the spline points vector
            spline_pts_x.push_back(prev_car_x);
            spline_pts_x.push_back(car_x);
            
            spline_pts_y.push_back(prev_car_y);
            spline_pts_y.push_back(car_y);
          }
          
          // If there are still a good number of points left in the previous path then use it's last two points to start the spline
          else
          {
            // Redefine the car reference point as the last point in the previous path
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];
            
            // Get the second to last point in the previous path
            double prev_ref_x = previous_path_x[prev_path_size - 2];
            double prev_ref_y = previous_path_y[prev_path_size - 2];
            // Get the new car yaw using the two points
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
            
            // Add the two points onto the spline vector
            spline_pts_x.push_back(prev_ref_x);
            spline_pts_x.push_back(ref_x);
            
            spline_pts_y.push_back(prev_ref_y);
            spline_pts_y.push_back(ref_y);
            
          }
          
          // Generate points along the current lane that you are in by incrementing the s value of the car
          vector<double> s_30_point = getXY(car_s+30, 4*lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> s_60_point = getXY(car_s+60, 4*lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> s_90_point = getXY(car_s+90, 4*lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          
          
          // Push the generated points onto the spline point vector
          spline_pts_x.push_back(s_30_point[0]);
          spline_pts_x.push_back(s_60_point[0]);
          spline_pts_x.push_back(s_90_point[0]);

          spline_pts_y.push_back(s_30_point[1]);
          spline_pts_y.push_back(s_60_point[1]);
          spline_pts_y.push_back(s_90_point[1]);
          
          // Transform all the point in the spline points vector into the car's frame of reference
          for(int i = 0; i < spline_pts_x.size(); i++)
          {
            // Shift the car's reference angle to 0
            double shift_x = spline_pts_x[i] - ref_x;
            double shift_y = spline_pts_y[i] - ref_y;
            
            spline_pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
            spline_pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
          }
          
          // Create a spline
          tk::spline sp;
          
          // Add the points to the spline
          sp.set_points(spline_pts_x, spline_pts_y);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Add the points from the previous path to the next path
          for(int i = 0; i < prev_path_size; i ++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Choose target distance in front of the car
          double target_x = 30.0;
          double target_y = sp(target_x);
          // Calculate the distance using the pythagoreum theorum
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
          
          // x location in front of the car. Will be updated for every new point
          double x_loc = 0.0;
          
          // Determine the number of points that need to be generated to cover the distance
          double N = (target_dist/(.02*target_vel/2.24)); // Divide velocity by 2.24 to convert from mph to meters per second
          
          // Calculate the x increment by dividing the total distance that needs to be covered by the number of points that will be generated across that distance
          double x_inc = target_x/N;
          
          // Fill in the missing points using the newly generated points
          for(int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            // Generate the next x point by adding the increment to the previous location
            double x_point = x_loc + x_inc;
            
            // Calculate the y value associated with the x input on the spline
            double y_point = sp(x_point);
            
            // Update the x location with the newly calculated point
            x_loc = x_point;
            
            // DIFFERENT FROM REF_X AND REF_Y. ref_x and ref_y should hold the first points on the path whether they are from the car's position or the las point in the previous path
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Untransform the points so they're in the global coordinate frame
            // Rotation
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
            
            // Translation
            x_point += ref_x;
            y_point += ref_y;
            
            
            
            // Add the point to the point lists
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
            
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