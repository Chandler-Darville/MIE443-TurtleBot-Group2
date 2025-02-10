#ifndef initializationHeader
#define initializationHeader

struct PathInfo {
    float heading;  // Optimal heading in degrees
    float distance; // Corresponding maximum distance
};



PathInfo findOptimalPath(const std::vector<float>& scanData);

#endif