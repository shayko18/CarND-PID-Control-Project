#define _USE_MATH_DEFINES

#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::stringstream hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_last_of("]");
	if (found_null != std::string::npos) {
		return std::stringstream();
	}
	else if (b1 != std::string::npos && b2 != std::string::npos) {
		std::stringstream tmp = std::stringstream();
		tmp.str(s.substr(b1, b2 - b1 + 1));
		return tmp;
	}
	return std::stringstream();
}

/*
#######################################   Describe the effect each components & how the final hyperparameters were chosen:  #######################################

1) I started it very low Kp (~0.01) and I set Ki nand Kd to zero.
2) I saw that the car didn't react fast enough to the curves on the road.
3) I increased Kp (~0.1), to react faster to the curves. Kp effects how quickly we react.
4) The car started to oscillate and was unstable.
5) I put small value to Ki (~0.0001), and a larger one to Kd (~1). Kd can help as fight the oscillations and Ki helps us to close a error from a drift but can cause overshoot.
6) I decreased the throttle value (in order to decreased the speed) when there was a big error magnitude (=fabs(cte)).
- This is like a human driver would do - when there is a sharp turn or when we start to lose control of the car we decreased our speed).
7) I saw that the car still hit the lane line on the sharp turns so I increased Kp a little more.
8) in order to reduce the oscillations, so the car will drive smoothly (not just not touching the lane lines) I played some more with Ki and Kd and the throttle value.
Below you can see my final values:
* PID: Kp=0.17, Ki=0.00008, Kd=1.1
* Throttle: low error=0.2, mid error=0.1, large error=0.05

######################################################################################################################################################################
*/

int main()
{
	uWS::Hub h;


	PID pid;
	pid.Init(0.17, 0.00008, 1.1);


	h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event

		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(std::string(data));
			if (s.str() != "") {
				auto j = json::parse(s);
				std::string event = j[0].get<std::string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					double cte = std::stod(j[1]["cte"].get<std::string>());
					double speed = std::stod(j[1]["speed"].get<std::string>());
					double angle = std::stod(j[1]["steering_angle"].get<std::string>());
					double steer_value;
					/*
					* TODO: Calcuate steering value here, remember the steering value is
					* [-1, 1].
					* NOTE: Feel free to play around with the throttle and speed. Maybe use
					* another PID controller to control the speed!
					*/

					//
					// Update the error according to the cte
					pid.UpdateError(cte);

					//
					// Write the steer_value according to the pid total error. The cte "wanted value" is 0.0 
					steer_value = 0.0 - pid.TotalError();

					//
					// Hard limit the steer value to [-1, 1]
					if (steer_value > 1.0) {
						steer_value = 1.0;
					}
					else if (steer_value < -1.0) {
						steer_value = -1.0;
					}

					//
					// set the throttle value. On big error magnitude we want to slow down (see point (6))
					double throttle_value;
					if ((cte > 0.8) || (cte < -0.8)) {        // high error magnitude
						throttle_value = 0.05;
					}
					else if ((cte > 0.3) || (cte < -0.3)) {   // mid error magnitude
						throttle_value = 0.1;
					}
					else {                                     // low error magnitude
						throttle_value = 0.2;
					}


					// DEBUG
					std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << " Speed: " << speed << std::endl;

					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle_value;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					//std::cout << msg << std::endl;
					std::cout << std::endl;
					(*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				(*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
		(*ws).close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen("0.0.0.0", port))
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