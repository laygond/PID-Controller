#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  //Create hub instance to connect to simulator
  uWS::Hub h;

  // Initialize steering pid object
  PID steering_controller;
  //double steer_Kp = 0.1, steer_Ki = 0.0001, steer_Kd = 1.0;
//   double steer_Kp = 0.15, steer_Ki = 0.001, steer_Kd = 1.0;
  double steer_Kp = 0.14, steer_Ki = 0.0009, steer_Kd = 1.0;
  steering_controller.Init(steer_Kp, steer_Ki, steer_Kd);

  // Initialize speed pid object
  PID speed_controller;
  double speed_Kp = 0.1, speed_Ki = 0.002, speed_Kd = 0.0; //taken from Behavioral Cloning project
  speed_controller.Init(speed_Kp, speed_Ki, speed_Kd);


  h.onMessage([&steering_controller, &speed_controller](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Read Feedback data
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          
          // Controller Updates Corrections
          double desired = 30.0; //mph
          double throttle_value = speed_controller.Update(speed - desired); 
          double steer_value    = steering_controller.Update(cte); // between  [-1, 1].
          
          // DEBUG
          std::cout 
          << "Input" << std::endl
          << "  CTE  : " << cte   << std::endl
          << "  Speed: " << speed << std::endl
          << "  Angle: " << angle << std::endl
          << "Output" << std::endl
          << "  Steer   : " << steer_value << std::endl
          << "  Throttle: " << throttle_value << std::endl
          << " ----- " << std::endl;
          
          // Send message back to simulator
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } 
      else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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