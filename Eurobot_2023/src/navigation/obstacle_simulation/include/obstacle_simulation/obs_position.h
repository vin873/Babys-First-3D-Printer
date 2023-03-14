#ifndef OBS_POSITION_H_
#define OBS_POSITION_H_

enum class Motion_type 
{
    STATIC,
    RECIPROCATION
};

class Obstacle
{
public:

    std::string from;
    Motion_type motion_type;
    std::vector<std::vector<double>> position;
    double stdev;
private:

};

#endif // OBS_POSITION_H