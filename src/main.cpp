#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// For convenience
using json = nlohmann::json;

namespace
{
    constexpr const double kMaxSpeed = 49.5;
    constexpr const double kMaxAcc = 0.224;
    
    // The max s value before wrapping around the track back to 0
    constexpr const double kMaxS = 6945.554;
    
    // For converting back and forth between radians and degrees.
    constexpr double pi()
    {
        return M_PI;
    }
    
    double deg2rad(double x)
    {
        return x * pi() / 180;
    }
    
    double rad2deg(double x)
    {
        return x * 180 / pi();
    }

    // Checks if the SocketIO event has JSON data.
    // If there is data the JSON object in string format will be returned,
    // else the empty string "" will be returned.
    string hasData(string s)
    {
        auto found_null = s.find("null");
        auto b1 = s.find_first_of("[");
        auto b2 = s.find_first_of("}");
        if (found_null != string::npos)
        {
            return "";
        }
        else if (b1 != string::npos && b2 != string::npos)
        {
            return s.substr(b1, b2 - b1 + 2);
        }
        
        return "";
    }

    double distance(double x1, double y1, double x2, double y2)
    {
        return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    }

    int ClosestWaypoint(double x, double y, const vector<double>& maps_x, const vector<double>& maps_y)
    {
        double closestLen = 100000; // Large number
        int closestWaypoint = 0;
        
        for(int i = 0; i < maps_x.size(); i++)
        {
            double map_x = maps_x[i];
            double map_y = maps_y[i];
            double dist = distance(x, y, map_x, map_y);
            if(dist < closestLen)
            {
                closestLen = dist;
                closestWaypoint = i;
            }
        }
        
        return closestWaypoint;
    }

    int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
    {
        int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);
        
        double map_x = maps_x[closestWaypoint];
        double map_y = maps_y[closestWaypoint];
        
        double heading = atan2((map_y-y), (map_x-x));
        double angle = abs(theta - heading);
        if(angle > pi()/4)
        {
            closestWaypoint++;
        }
        
        return closestWaypoint;
    }

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
    {
        int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
        
        int prev_wp;
        prev_wp = next_wp-1;
        if(next_wp == 0)
        {
            prev_wp  = maps_x.size()-1;
        }
        
        double n_x = maps_x[next_wp]-maps_x[prev_wp];
        double n_y = maps_y[next_wp]-maps_y[prev_wp];
        double x_x = x - maps_x[prev_wp];
        double x_y = y - maps_y[prev_wp];
        
        // Find the projection of x onto n
        double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
        double proj_x = proj_norm * n_x;
        double proj_y = proj_norm * n_y;
        
        double frenet_d = distance(x_x, x_y, proj_x, proj_y);
        
        // See if d value is positive or negative by comparing it to a center point
        double center_x = 1000-maps_x[prev_wp];
        double center_y = 2000-maps_y[prev_wp];
        double centerToPos = distance(center_x,center_y,x_x,x_y);
        double centerToRef = distance(center_x,center_y,proj_x,proj_y);
        if(centerToPos <= centerToRef)
        {
            frenet_d *= -1;
        }
        
        // Calculate s value
        double frenet_s = 0;
        for(int i = 0; i < prev_wp; i++)
        {
            frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
        }
        
        frenet_s += distance(0, 0, proj_x, proj_y);
        return {frenet_s, frenet_d};
    }

    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
    {
        int prev_wp = -1;
        
        while((s > maps_s[prev_wp + 1]) && (prev_wp < (int)(maps_s.size() - 1)))
        {
            prev_wp++;
        }
        
        int wp2 = (prev_wp + 1) % maps_x.size();
        double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
        
        // The x,y,s along the segment
        double seg_s = (s - maps_s[prev_wp]);
        double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
        double seg_y = maps_y[prev_wp] + seg_s*sin(heading);
        
        double perp_heading = heading - (pi() / 2);
        double x = seg_x + (d * cos(perp_heading));
        double y = seg_y + (d * sin(perp_heading));
        
        return {x,y};
    }
    
    struct CarInfo
    {
        uint32_t lane;
        
        double car_x;
        double car_y;
        
        double car_s;
        double car_d;
        
        double car_yaw;
        double car_speed;
        double speed_diff;
        
        double ref_vel;
    };
    
    struct PathInfo
    {
        uint32_t prev_size;
        
        // Previous path data given to the Planner
        vector<double> previous_path_x;
        vector<double> previous_path_y;
        
        // Previous path's end s and d values
        double end_path_s;
        double end_path_d;
    };
    
    struct MapWayPoints
    {
        vector<double> x;
        vector<double> y;
        vector<double> s;
        vector<double> dx;
        vector<double> dy;
    };
}

int main()
{
    uWS::Hub h;
    
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    MapWayPoints map_waypoints;
    
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    
    ifstream in_map_(map_file_.c_str(), ifstream::in);
    
    string line;
    while (getline(in_map_, line))
    {
        istringstream iss(line);
        
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
        
        map_waypoints.x.push_back(x);
        map_waypoints.y.push_back(y);
        map_waypoints.s.push_back(s);
        map_waypoints.dx.push_back(d_x);
        map_waypoints.dy.push_back(d_y);
    }
    
    // Car's lane, starting at the middle lane
    int lane = 1;
    
    // Reference velocity, unit: mph
    double ref_vel = 0.0;
    
    h.onMessage([&ref_vel, &lane, &map_waypoints]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(data);
            if (s == "")
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                return;
            }
            
            auto j = json::parse(s);
            string event = j[0].get<string>();
            if (event == "telemetry")
            {
                // j[1] is the data JSON object
                
                // Main car's localization Data
                CarInfo car_info;
                car_info.lane = lane;
                car_info.ref_vel = ref_vel;
                car_info.car_x = j[1]["x"];
                car_info.car_y = j[1]["y"];
                car_info.car_s = j[1]["s"];
                car_info.car_d = j[1]["d"];
                car_info.car_yaw = j[1]["yaw"];
                car_info.car_speed = j[1]["speed"];
                car_info.speed_diff = 0;
                
                // Previous path data given to the Planner
                PathInfo path_info;
                path_info.previous_path_x = vector<double>(j[1]["previous_path_x"].begin(), j[1]["previous_path_x"].end());
                path_info.previous_path_y = vector<double>(j[1]["previous_path_y"].begin(), j[1]["previous_path_y"].end());
                
                // Provided previous path point size
                path_info.prev_size = path_info.previous_path_x.size();
                
                // Previous path's end s and d values
                path_info.end_path_s = j[1]["end_path_s"];
                path_info.end_path_d = j[1]["end_path_d"];
                
                // Sensor fusion data, a list of all other cars on the same side of the road
                auto sensor_fusion = j[1]["sensor_fusion"];
                
                // Preventing collitions
                if (path_info.prev_size > 0)
                {
                    car_info.car_s = path_info.end_path_s;
                }
                
                // Prediction: Analysing other cars positions
                bool is_car_ahead = false;
                bool is_car_on_left = false;
                bool is_car_on_right = false;
                
                //cout << "____________________________" << endl;
                //cout << "lane: " << car_info.lane << ", car_info.car_s: " << car_s << endl;
                uint32_t nearest_car_ahead_on_left = car_info.car_s + kMaxS;
                uint32_t nearest_car_ahead_on_right = car_info.car_s + kMaxS;
                
                for (int i = 0; i < sensor_fusion.size(); i++)
                {
                    float d = sensor_fusion[i][6];
                    int car_lane_to_check = -1;
                    
                    // Check whether it is on the same lane we are
                    if (d > 0 && d < 4)
                    {
                        car_lane_to_check = 0;
                    }
                    else if (d > 4 && d < 8)
                    {
                        car_lane_to_check = 1;
                    }
                    else if (d > 8 && d < 12)
                    {
                        car_lane_to_check = 2;
                    }
                    
                    if (car_lane_to_check < 0)
                    {
                        continue;
                    }
                    
                    // Find car speed
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double car_speed_to_check = sqrt(vx*vx + vy*vy);
                    double car_s_to_check = sensor_fusion[i][5];
                    //cout << "check_lane: " << car_lane_to_check << ", car_s_to_check: " << car_s_to_check << endl;
                    
                    // Estimate car s position after executing previous trajectory
                    uint32_t lane_switching_buffer_ahead_me = 25;
                    uint32_t lane_switching_buffer_behind_me = 15;
                    
                    car_s_to_check += (double(path_info.prev_size) * 0.02 * car_speed_to_check);
                    if (car_lane_to_check == car_info.lane)
                    {
                        // Car is in our lane
                        is_car_ahead |= (car_s_to_check > car_info.car_s) && (car_s_to_check < car_info.car_s + lane_switching_buffer_ahead_me);
                    }
                    else if (car_lane_to_check - car_info.lane == -1)
                    {
                        // Car is on the left
                        is_car_on_left |= (car_s_to_check > car_info.car_s - lane_switching_buffer_behind_me) && (car_s_to_check < car_info.car_s + lane_switching_buffer_ahead_me);
                        
                        if (!is_car_on_left)
                        {
                            //cout << "left lane: free, " << "nearest_car_ahead_on_left: " << nearest_car_ahead_on_left << endl;
                            if ((car_s_to_check > car_info.car_s) && (nearest_car_ahead_on_left > car_s_to_check))
                            {
                                nearest_car_ahead_on_left = car_s_to_check;
                                //cout << "nearest_car_ahead_on_left: " << nearest_car_ahead_on_left << endl;
                            }
                        }
                    }
                    else if (car_lane_to_check - car_info.lane == 1)
                    {
                        // Car is on the right
                        is_car_on_right |= (car_s_to_check > car_info.car_s - lane_switching_buffer_behind_me) && (car_s_to_check < car_info.car_s + lane_switching_buffer_ahead_me);
                        
                        if (!is_car_on_right)
                        {
                            //cout << "right lane: free, " << "nearest_car_ahead_on_right: " << nearest_car_ahead_on_right << endl;
                            if ((car_s_to_check > car_info.car_s) && (nearest_car_ahead_on_right > car_s_to_check))
                            {
                                nearest_car_ahead_on_right = car_s_to_check;
                                //cout << "nearest_car_ahead_on_right: " << nearest_car_ahead_on_right << endl;
                            }
                        }
                    }
                }
                
                // Behavior: Let's see what to do
                if (is_car_ahead)
                {
                    bool is_left_lane_free = !is_car_on_left && (car_info.lane > 0);
                    bool is_right_lane_free = !is_car_on_right && (car_info.lane != 2);
                    if (is_left_lane_free && is_right_lane_free)
                    {
                        // Check which lane is better - less traffic
                        if (nearest_car_ahead_on_right > nearest_car_ahead_on_left)
                        {
                            cout << "Both lanes are free, the right lane is better, " << "nearest_car_ahead_on_left: " << nearest_car_ahead_on_left << ", nearest_car_ahead_on_right: " << nearest_car_ahead_on_right << endl;
                            car_info.lane++;
                        }
                        else
                        {
                            car_info.lane--;
                        }
                    }
                    else if (is_left_lane_free)
                    {
                        // If there is no car on the left and there is a left lane, switch to the left lane
                        car_info.lane--;
                    }
                    else if (is_right_lane_free)
                    {
                        // If there is no car on the right and there is a right lane, switch to the right lane
                        car_info.lane++;
                    }
                    else
                    {
                        car_info.speed_diff -= kMaxAcc;
                    }
                }
                else
                {
                    if (car_info.ref_vel < kMaxSpeed)
                    {
                        car_info.speed_diff += kMaxAcc;
                    }
                }
                
                vector<double> ptsx;
                vector<double> ptsy;
                
                double ref_x = car_info.car_x;
                double ref_y = car_info.car_y;
                double ref_yaw = deg2rad(car_info.car_yaw);
                
                // Check whether we have previous points
                if (path_info.prev_size < 2)
                {
                    // There are not too many
                    double prev_car_x = car_info.car_x - cos(car_info.car_yaw);
                    double prev_car_y = car_info.car_y - sin(car_info.car_yaw);
                    
                    ptsx.push_back(prev_car_x);
                    ptsx.push_back(car_info.car_x);
                    
                    ptsy.push_back(prev_car_y);
                    ptsy.push_back(car_info.car_y);
                }
                else
                {
                    // Use the last two points
                    ref_x = path_info.previous_path_x[path_info.prev_size - 1];
                    ref_y = path_info.previous_path_y[path_info.prev_size - 1];
                    
                    double ref_x_prev = path_info.previous_path_x[path_info.prev_size - 2];
                    double ref_y_prev = path_info.previous_path_y[path_info.prev_size - 2];
                    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
                    
                    ptsx.push_back(ref_x_prev);
                    ptsx.push_back(ref_x);
                    
                    ptsy.push_back(ref_y_prev);
                    ptsy.push_back(ref_y);
                }
                
                // Set up the target points in the future
                vector<double> next_wp0 = getXY(car_info.car_s + 30, 2 + 4*car_info.lane, map_waypoints.s, map_waypoints.x, map_waypoints.y);
                vector<double> next_wp1 = getXY(car_info.car_s + 60, 2 + 4*car_info.lane, map_waypoints.s, map_waypoints.x, map_waypoints.y);
                vector<double> next_wp2 = getXY(car_info.car_s + 90, 2 + 4*car_info.lane, map_waypoints.s, map_waypoints.x, map_waypoints.y);
                
                ptsx.push_back(next_wp0[0]);
                ptsx.push_back(next_wp1[0]);
                ptsx.push_back(next_wp2[0]);
                
                ptsy.push_back(next_wp0[1]);
                ptsy.push_back(next_wp1[1]);
                ptsy.push_back(next_wp2[1]);
                
                // Make coordinates to local car coordinates
                for (int i = 0; i < ptsx.size(); i++)
                {
                    double shift_x = ptsx[i] - ref_x;
                    double shift_y = ptsy[i] - ref_y;
                    
                    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                }
                
                // Create the spline
                tk::spline s;
                s.set_points(ptsx, ptsy);
                
                // Output path points from previous path for continuity
                vector<double> next_x_vals;
                vector<double> next_y_vals;
                for (int i = 0; i < path_info.prev_size; i++)
                {
                    next_x_vals.push_back(path_info.previous_path_x[i]);
                    next_y_vals.push_back(path_info.previous_path_y[i]);
                }
                
                // Calculate distance y position on 30m ahead
                double target_x = 30.0;
                double target_y = s(target_x);
                double target_dist = sqrt(target_x*target_x + target_y*target_y);
                
                double x_add_on = 0;
                
                for(int i = 1; i < 50 - path_info.prev_size; i++)
                {
                    car_info.ref_vel += car_info.speed_diff;
                    if (car_info.ref_vel > kMaxSpeed)
                    {
                        car_info.ref_vel = kMaxSpeed;
                    }
                    else if (car_info.ref_vel < kMaxAcc)
                    {
                        car_info.ref_vel = kMaxAcc;
                    }
                    
                    double N = target_dist / (0.02 * car_info.ref_vel / 2.24);
                    double x_point = x_add_on + (target_x / N);
                    double y_point = s(x_point);
                    
                    x_add_on = x_point;
                    
                    double x_ref = x_point;
                    double y_ref = y_point;
                    
                    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
                    
                    x_point += ref_x;
                    y_point += ref_y;
                    
                    next_x_vals.push_back(x_point);
                    next_y_vals.push_back(y_point);
                }
                
                lane = car_info.lane;
                ref_vel = car_info.ref_vel;
                
                json msgJson;
                msgJson["next_x"] = next_x_vals;
                msgJson["next_y"] = next_y_vals;
                auto msg = "42[\"control\","+ msgJson.dump()+"]";
                //this_thread::sleep_for(chrono::milliseconds(1000));
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // We don't need this since we're not using HTTP but if it's removed the
    // program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // I guess this should be done more gracefully?
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
