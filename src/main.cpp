#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // The global x positions of the waypoints.
          vector<double> ptsx = j[1]["ptsx"];
          
          // The global y positions of the waypoints.
          // This corresponds to the z coordinate in Unity since y is the up-down direction.
          vector<double> ptsy = j[1]["ptsy"];
          
          // The global x, y position of the vehicle.
          double px = j[1]["x"];
          double py = j[1]["y"];
          
          // The orientation of the vehicle in radians converted from the Unity format to the standard format
          double psi = j[1]["psi"];
          
          // The current velocity in mph.
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // The current steering angle in radians.
          double steer_value    = j[1]["steering_angle"];
          
          // The current throttle value [-1, 1].
          double throttle_value = j[1]["throttle"];

          // Remember that the server returns waypoints using the map's coordinate system,
          // which is different than the car's coordinate system. Transforming these waypoints
          // will make it easier to both display them and to calculate the CTE and Epsi values
          // for the model predictive controller.
          Eigen::VectorXd ptsx_car_coord(ptsx.size());
          Eigen::VectorXd ptsy_car_coord(ptsy.size());
          
          for (int i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            ptsx_car_coord[i] = dx * cos(psi) + dy * sin(psi);
            ptsy_car_coord[i] = dy * cos(psi) - dx * sin(psi);
          }
          
          // fit a polynomial to the above x and y coordinates. Use 3rd degree polynomial since it is not a straight line
          auto coeffs = polyfit(ptsx_car_coord, ptsy_car_coord, 3);
          
          
          // NOTE: free feel to play around with these
          double x = -1;
          double y = 10;
          double psi_ = 0;
          
          // calculate the cross track error
          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          double cte = polyeval(coeffs, x) - y;
          
          // calculate the orientation error
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = psi_ - atan(coeffs[1]);
          
          double latency_dt = 0.1;
          const double Lf = 2.67;
          x = x+1;
          y = y-10;
          v = v + throttle_value * latency_dt;
          cte = cte + v * sin(epsi) * latency_dt;
          epsi = epsi + v * steer_value / Lf * latency_dt;

          Eigen::VectorXd state(6);
          state << x, y, psi_, v, cte, epsi;
          
          auto vars = mpc.Solve(state, coeffs);
          
          json msgJson;
          
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          steer_value    = -1 * (vars[0]/deg2rad(25));
          throttle_value = vars[1];
          
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"]       = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (int i = 2, index = 0; i < (vars.size()-2); i+=2) {
            index = i;
            mpc_x_vals.push_back(vars[index++]);
            mpc_y_vals.push_back(vars[index]);
          }
          
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          assert(ptsx_car_coord.size() == ptsy_car_coord.size());
          for (int i = 0; i < ptsx_car_coord.size(); i++) {
            next_x_vals.push_back(ptsx_car_coord[i]);
            next_y_vals.push_back(ptsy_car_coord[i]);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
