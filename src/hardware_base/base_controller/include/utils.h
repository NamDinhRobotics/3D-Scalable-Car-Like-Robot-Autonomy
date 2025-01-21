//
// Created by dinhnambkhn on 27/11/2024.
//

#ifndef UGV_AUTONOMY_WS_UTILS_H
#define UGV_AUTONOMY_WS_UTILS_H

struct Waypoint {
    int id = 0;              // Default value for id
    double x = 0.0;          // Default x-coordinate
    double y = 0.0;          // Default y-coordinate
    double z = 0.0;          // Default z-coordinate
    double qx = 0.0;         // Default quaternion x
    double qy = 0.0;         // Default quaternion y
    double qz = 0.0;         // Default quaternion z
    double qw = 1.0;         // Default quaternion w (identity rotation)
    double yaw = 0.0;        // Default yaw angle
    double curvature = 0.0;  // Default curvature
    double ref_speed = 0.0;  // Default reference speed
};


#endif //UGV_AUTONOMY_WS_UTILS_H
