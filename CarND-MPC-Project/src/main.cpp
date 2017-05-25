#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "MPC.h"
#include "json.hpp"
#include "helper_functions.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double MAX_ERROR = 999999;

static unsigned int last_now = 0;

double steer_value;
double throttle_value;

vector<double> mpc_x;
vector<double> mpc_y;
vector<double> next_x;
vector<double> next_y;

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
//		cout << sdata << endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			string s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					vector<double> ptsx = j[1]["ptsx"];
					vector<double> ptsy = j[1]["ptsy"];

					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					double v = j[1]["speed"];

					Eigen::VectorXd conv_state(3);
					conv_state << px, py, psi;
//				    std::cout <<"conv_state:" << conv_state.transpose() << std::endl;

					//latency corrected car pos
					Eigen::VectorXd corrected_pos(3);
					Eigen::VectorXd corrected_map_pos(3);
					//adapt position because of latency since last command
					//center the way point line
					corrected_pos << (v*0.06), 0.9, 0;
					corrected_map_pos = convertCarToMap(conv_state,corrected_pos);

					conv_state << corrected_map_pos[0], corrected_map_pos[1], psi;
//					std::cout <<"conv_state:" << conv_state.transpose() << std::endl;


					//convert to car coordinate system. x straigh, y left
					vector<double> car_x = vector<double>();
					vector<double> car_y = vector<double>();
					convertVectorMapToCar(conv_state,ptsx,ptsy,car_x,car_y, false);

					//polynom fit
					int order = min(5, (int)car_x.size()-1);
					Eigen::VectorXd coeffs = polyfit(toVectorXd(car_x),toVectorXd(car_y),order);
					//calc cte, x == 0
					double cte =  polyeval(coeffs, 0);
					double epsi = -atan(coeffs[1]);

					//reduce order if the polyfit solution is off
					while (abs(cte)>1 && order > 3)
					{
						std::cout <<"bad poly fit, reduce order to " << --order << std::endl;
						coeffs = polyfit(toVectorXd(car_x),toVectorXd(car_y),order);
						cte =  polyeval(coeffs, 0);
						epsi = -atan(coeffs[1]);
					}

					//epsi = coeff[1] because of y == 0

					Eigen::VectorXd state(6);
					state << 0, 0, 0, v, cte, epsi;

					//increase N if the car is slow
				    size_t N = 10;
				    if(v < 10)
						N = 45;

					std::cout <<"state:" << state.transpose() << " N:" << N << " coeffs:" << coeffs.transpose() << std::endl;
					auto vars = mpc.Solve(state, coeffs,N);

					Eigen::VectorXd solution(8);
					solution << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5], vars[6], vars[7];
					if(vars[10] != 1)
					{
					    std::cout <<"bad solution:" << solution.transpose() << std::endl;

					} else {
					    std::cout <<"solution:" << solution.transpose() << std::endl;

					    //average steering and throttle
						steer_value = (steer_value  -vars[6])*0.5;
						throttle_value = (throttle_value + vars[7])*0.5;

						//paint prediction
						mpc_x.clear();
						mpc_y.clear();
						mpc_x.push_back(vars[0]);
						mpc_y.push_back(vars[1]);
						for(unsigned int i =0;i < 5;i++)
						{
							mpc_x.push_back(vars[11+2*i]);
							mpc_y.push_back(vars[12+2*i]);
						}
					}

					 //Calculate steeering angle and throttle using MPC.
//					std::cout << "steering:" << steer_value <<" throttle:"<<throttle_value << endl;

					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle_value;

					//paint ground truth waypoints
					next_x.clear();
					next_y.clear();
					for(unsigned int i =0;i < car_x.size();i++)
					{
						next_x.push_back(car_x[i]);
						next_y.push_back(car_y[i]);
					}

					msgJson["mpc_x"]  = mpc_x;
					msgJson["mpc_y"]  = mpc_y;
					msgJson["next_x"]  = next_x;
					msgJson["next_y"]  = next_y;

					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//					std::cout << msg << std::endl;

					// Latency
					// The purpose is to mimic real driving conditions where
					// the car does actuate the commands instantly.
					unsigned int now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
				    unsigned int latency = now - last_now;

				    //add wait states to get in total to 100 milisecond latency
					int wait = 100 - latency;
					if(wait > 100)wait=100;
					if(wait < 0)wait=0;

				    std::cout << "latency:" << latency << " wait:" << wait << std::endl;
					this_thread::sleep_for(chrono::milliseconds(wait));
					last_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			} else {
				// Manual drivinges to draw
				int num_particles;
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
