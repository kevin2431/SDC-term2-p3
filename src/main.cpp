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
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  //pid系数
  double p[3]={0.552,0.00082,8.216};
  double dp[3]={0.22,0.001,0.53};
  int p_it=2;
  int n=0;
  bool direction=true;
  bool twiddle=false;
  double best_err=100000.0;
  double total_err=0;
  if(twiddle) 
    pid.Init(p[0],p[1],p[2]);
  else pid.Init(0.552,0.00001,8.1577);

  h.onMessage([&pid, &p, &dp, &direction, &p_it, &twiddle, &best_err, &total_err, &n](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          //使用pid 控制speed 即 throttle
          

          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          //double throttle=0.24*exp(-fabs(steer_value))+0.02;
          double throttle=0.4;

          double tol=0.2;
          int max_n=2200;

          if(twiddle) {
            if(n>100) 
              //total_err+=pow(cte,2);
              total_err+=fabs(cte);
            if(n==0) pid.Init(p[0],p[1],p[2]);
            n++;

            //超出调试步数
            if(n>max_n){
              total_err/=((max_n-100));
              total_err*=10000.0;
              //正方向增长
              if(direction){
                if(total_err<best_err){
                  best_err=total_err;
                  dp[p_it]*=1.1;
                  p_it=(p_it+1)%3;
                  //更新p
                  p[p_it]+=dp[p_it];
                }else{
                  direction=false;
                  p[p_it]-=(2*dp[p_it]);
                }
              }else{
                if(total_err<best_err){
                  best_err=total_err;
                  dp[p_it]*=1.1;
                }else{
                  p[p_it]+=dp[p_it];
                  dp[p_it]*=0.9;
                }
                p_it=(p_it+1)%3;
                p[p_it]+=dp[p_it];
                direction=true;
              }
              //rest整个模拟器
              double sum_dp=dp[0]+dp[1]+dp[2];
              std::cout<<"err:"<<total_err<<"  best_err:"<<best_err<<std::endl;
              if(sum_dp<tol) {
                twiddle=false;
                //打印最优信息
                std::cout<<"达到最优"<<std::endl;

                //暂停，等待一个按键
              }
              n=0;
              total_err=0;
              //先将角度和速度置0
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

            }else{
              //正常进行下一步
              std::cout<<"err:"<<best_err<<std::endl;
              std::cout<<"p:"<<p[0]<<" i:"<<p[1]<<" d:"<<p[2]<<std::endl;
              std::cout<<"dp:"<<dp[0]<<" di:"<<dp[1]<<" dd:"<<dp[2]<<std::endl;
              std::cout<<(dp[0]+dp[1]+dp[2])<<std::endl;
              std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            }
          }else{
            //不需要twiddle时
            
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                      << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }  // end "telemetry" if
        }
      } else {
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