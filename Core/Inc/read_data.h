#include "Datatype/vector.h"
#include "cstdint"

class Read_data
{
    int index; //which element of std::vector
    int size_of_data;
    bool accessible;
    bool skip_indexIcrease;
    uint8_t temp[44];
    float depth;
    float yaw;
    geometry::Vector v;
    float velocity[3];
    float joint[3];

public:
    uint8_t ch;
    Read_data();
    float get_single_num();
    void assign_num();
    void receieve();
    int get_size_of_data(){return size_of_data;};

    float get_depth(){return depth;};
    float get_yaw(){return yaw;};
    bool access_ok(){return accessible;};
    void skip_indexIncrease_init(){skip_indexIcrease = false;};
    void access_init(){accessible = false;};
    geometry::Vector get_geometry_vector(){return v;};
    float get_joint0(){return joint[0];};
    float get_joint1(){return joint[1];};
    float get_joint2(){return joint[2];};
    float get_vel0(){return velocity[0];};
    float get_vel1(){return velocity[1];};
    float get_vel2(){return velocity[2];};
};
