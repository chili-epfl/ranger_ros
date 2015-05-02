
#ifndef ASEBACONNECTOR_H
#define ASEBACONNECTOR_H

#include <vector>
#include <string>

#include <dashel/dashel.h>
#include <aseba/common/msg/descriptions-manager.h>

const int RANGER_MAIN_FEEDBACK_WITH_ENCODERS_EVENT=1;
const int RANGER_SET_SPEED_EVENT=4;
const int ENABLE_ENCODERS=5;
const int ENABLE_FEEDBACK=12;
// constants for linear scale interpolation
const int SCALE_EMPTY_RAW_VALUE = 6000;
const double SCALE_KG_PER_RAW_UNIT = 0.0008755615;

//const int SCALE_REFS_NROWS = 7;
//const int SCALE_REFS[SCALE_REFS_NROWS][2] =
//{
//    {0,     4850},
//    {0.123, 4980},
//    {0.325, 5220},
//    {0.972, 5940},
//    {2.505, 7750},
//    {4.505, 10040},
//    {6.630, 12550}
//};

struct RangerAsebaBridge: public Dashel::Hub, public Aseba::DescriptionsManager
{

protected:
    Dashel::Stream* targetStream;

    // result of last compilation and load used to interprete messages
    Aseba::CommonDefinitions commonDefinitions;

    float scaleInterpolation(int val) const;


public:
    // interface with main()
    RangerAsebaBridge(const char* target);
    bool isValid() const;

    // Ranger specific
    int l_encoder, r_encoder;
    float scale;		// measured weight, in kg
    float voltage;		// battery level, in mV
    bool is_charging;

    void setSpeed(int l_wheel, int r_wheel);

protected:
    // reimplemented from parent classes
    virtual void incomingData(Dashel::Stream *stream);

    void emit(int event, std::vector<int> args);
    void emit(int event, const std::vector<std::string>& args);
};

#endif
