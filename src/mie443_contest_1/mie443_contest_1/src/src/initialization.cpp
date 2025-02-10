#include <initialization.h>
#include <ros/console.h>

float init_scan[960];

const float scan_fov = 58.0;        // degrees

const int ranges_size = 640;

const float index_res = scan_fov/ranges_size;

int collectLaserScan(float ranges[ranges_size], int pos) {     // pos = 0,1,2,3,4,5
    // function assumes starting angle is 0deg and fov 60deg.
    // need to rotate 30deg before scanning so that init_scan[0] aligns with ranges[0]

    int scan_i = 0;

    if (pos > 0 && pos < 6) {scan_i = 160*pos;}
    else {
        ROS_INFO("Error: pos out of range (collectLaserScan)");
    }

    for (int i; i < 160; i++) {

        if (i % 4 == 0) {

            init_scan[scan_i + i] = ranges[i]

        }

    }

    return pos + 1

}

void fullScan(const sensor_msgs::LaserScan::ConstPtr& msgLaser) {

    int pos = 0;

    float ranges[ranges_size];

    while (pos < 6) {

        if (pos == 0) {
            // rotate 30 deg
        }
        else if (pos > 0 && pos < 6) {
            // rotate 60 deg
        }
        else {
            ROS_INFO("Error: pos out of range (fullScan)");
            break;
        }

        ranges = msgLaser->ranges;

        pos = collectLaserScan(ranges, pos);
    }

}


#include <vector>
#include <cmath>

struct PathInfo {
    float heading;  // Optimal heading in degrees
    float distance; // Corresponding maximum distance
};

PathInfo findOptimalPath(const std::vector<float>& scanData) {
    const int scanSize = 960;
    const float fieldOfView = 360.0f; // Full circle scan
    const float anglePerIndex = fieldOfView / scanSize;
    const float robotWidth = 0.34f; // 340 mm converted to meters

    // Convert robot width to an angular span
    float angularSpan = (robotWidth / (*std::max_element(scanData.begin(), scanData.end()))) * (180.0 / M_PI);
    int minSegmentSize = std::ceil(angularSpan / anglePerIndex);

    int bestStart = -1, bestLength = 0;
    float maxDistance = 0.0f;
    
    int currentStart = -1, currentLength = 0;
    for (int i = 0; i < scanSize; ++i) {
        if (scanData[i] > robotWidth) {
            if (currentStart == -1) currentStart = i;
            currentLength++;
        } else {
            if (currentLength >= minSegmentSize && scanData[currentStart + currentLength / 2] > maxDistance) {
                bestStart = currentStart;
                bestLength = currentLength;
                maxDistance = scanData[currentStart + currentLength / 2];
            }
            currentStart = -1;
            currentLength = 0;
        }
    }

    // Final check in case the best path is at the end
    if (currentLength >= minSegmentSize && scanData[currentStart + currentLength / 2] > maxDistance) {
        bestStart = currentStart;
        bestLength = currentLength;
        maxDistance = scanData[currentStart + currentLength / 2];
    }

    // Compute the best heading
    float bestHeading = (bestStart + bestLength / 2) * anglePerIndex;

    return {bestHeading, maxDistance};
}
