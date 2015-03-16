
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

struct RangerAsebaBridge: public Dashel::Hub, public Aseba::DescriptionsManager
{

protected:
    Dashel::Stream* targetStream;

    // result of last compilation and load used to interprete messages
    Aseba::CommonDefinitions commonDefinitions;

public:
    // interface with main()
    RangerAsebaBridge(const char* target);
    bool isValid() const;

    // Ranger specific
    int l_encoder, r_encoder;
    bool is_charging;

    void setSpeed(int l_wheel, int r_wheel);

protected:
    // reimplemented from parent classes
    virtual void incomingData(Dashel::Stream *stream);

    void emit(int event, std::vector<int> args);
    void emit(int event, const std::vector<std::string>& args);
};

#endif
