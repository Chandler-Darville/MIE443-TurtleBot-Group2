#include <initialization.h>
#include <ros/console.h>

float init_scan[960];

const float scan_fov = 58.0;        // degrees

const int ranges_size = 640;

const float index_res = scan_fov/ranges_size;

void collectLaserScan(float ranges[ranges_size], int pos) {     // pos = 0,1,2,3,4,5
    // function assumes starting angle is 0deg and fov 60deg.
    // need to rotate 30deg before scanning so that init_scan[0] aligns with ranges[0]

    int scan_i = 0;

    if (pos > 0 && pos < 6) {scan_i = 160*pos;}
    else {
        ROS_INFO("Error: pos out of range");
    }

    for (int i; i < 160; i++) {

        if (i % 4 == 0) {

            init_scan[scan_i + i] = ranges[i]

        }

        
    }

}