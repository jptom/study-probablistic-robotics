#ifndef _ENV_H_
#define _ENV_H_
#include <vector>



class Landmark {
public:
    Landmark(const double x, const double y)
    {
        pos.push_back(x);
        pos.push_back(y);
    }
    
public:
    std::vector<double> pos;
    size_t id;
};

class Map {
public:
    void append_landmark(const Landmark & lm){
        lm.id = landmarks.size();
        landmarks.push_back(lm);
    }
    Map & operator =(const Map & other){
        //if (this == &other) { return *this; }
        for (size_t i=0; i<other.landmarks.size(); ++i){
            append_landmark(Landmark(other.landmarks[i].pos[0], other.landmarks[i].pos[1]));
        }
        return *this;
    }
public:
    std::vector<Landmark> landmarks;
};

#endif
