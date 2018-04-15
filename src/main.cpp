#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "FuzzyPID.h"
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <chrono>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// For writing to a file
std::ofstream pid_output;

// For keeping time
using namespace std::chrono;
high_resolution_clock::time_point start_time = high_resolution_clock::now();

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char **argv)
{
  uWS::Hub h;

  // Open file to output to
  pid_output.open("./pid_output.csv");

  // Create PID object(s)
  PID steering_pid;
  FuzzyPID steering_fuzzy_pid;

  // The below values were determined manually
  double p = 0.1;
  double i = 0.0005;
  double d = 1.0;

  // To check other PID values and their effects on the driving, pass in all 3 of the PID values as follows
  // $ ./pid Kp Ki Kd
  // Initialize pid coefficients using passed in parameters
  std::cout << "Num args: " << argc << std::endl;
  if (argc == 4) {
    std::cout << "Using values (" << argv[1] << "," << argv[2] << "," << argv[3] << ")" << std::endl;
    p = std::atof(argv[1]);
    i = std::atof(argv[2]);
    d = std::atof(argv[3]);
  }

  // Initialize PID controller to the provided values
  steering_pid.Init(p, i, d);
  steering_fuzzy_pid.Init(p, i, d);

  h.onMessage([&steering_fuzzy_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double pos_x = std::stod(j[1]["x"].get<std::string>());
          double pos_y = std::stod(j[1]["y"].get<std::string>());
          double curr_wp_x = std::stod(j[1]["curr_wp_x"].get<std::string>());
          double curr_wp_y = std::stod(j[1]["curr_wp_y"].get<std::string>());
          double prev_wp_x = std::stod(j[1]["prev_wp_x"].get<std::string>());
          double prev_wp_y = std::stod(j[1]["prev_wp_y"].get<std::string>());
          double steer_value;
          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          //****************************************************************************************
          // Calculate Steering Angle
          //****************************************************************************************
          // Get Time info
          high_resolution_clock::time_point time_now = high_resolution_clock::now();
          duration<double, std::milli> time_passed = time_now - start_time;

          // // Update PID state
          // steering_pid.UpdateError(cte);
          steering_fuzzy_pid.UpdateError(cte, speed, time_passed.count());

          // // Get steering values
          // steer_value = steering_pid.TotalError();
          steer_value = steering_fuzzy_pid.TotalError();
          steer_value = std::min(1.0, std::max(-1.0, steer_value)); // Set it within the bounds of 1 and -1

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          // Output to file
          // Time, X, Y, CurrWP_X, CurrWP_Y, PrevWP_X, PrevWP_Y, CTE, Speed, Angle, Steer_Value, FuzzyPID_V_prev
          // pid_output << time_passed.count() << "," << cte << "," << speed << "," << angle << "," << steer_value << std::endl;
          pid_output << time_passed.count()
                     << "," << pos_x
                     << "," << pos_y
                     << "," << curr_wp_x
                     << "," << curr_wp_y
                     << "," << prev_wp_x
                     << "," << prev_wp_y
                     << "," << cte
                     << "," << speed
                     << "," << angle
                     << "," << steer_value
                     << "," << steering_fuzzy_pid.V_tm1
                     << "," << steering_fuzzy_pid.Kp
                     << std::endl;

          // Send data back to server
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
