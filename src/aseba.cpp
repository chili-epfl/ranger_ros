#include <iostream>
#include <sstream>
#include <iterator>
#include <cstdlib>
#include <algorithm> //std::copy

#include "aseba.h"

#include <ros/ros.h> // for logging only

using namespace std;
using namespace Dashel;
using namespace Aseba;


RangerAsebaBridge::RangerAsebaBridge(const char* target):
    targetStream(connect(target))
{
    // request a description of the target
    GetDescription getDescription;
    getDescription.serialize(targetStream);
    targetStream->flush();
    
    // emit event to enable feedback and encoders
	vector<int> arg; 
	arg.push_back(1);   
    emit(ENABLE_ENCODERS, arg);
    emit(ENABLE_FEEDBACK, arg);
}

bool RangerAsebaBridge::isValid() const
{
    return targetStream;
}

float RangerAsebaBridge::scaleInterpolation(int val) const
{
    float weightInKg= 0;

    if (val > SCALE_EMPTY_RAW_VALUE)
    {
        weightInKg = (val-SCALE_EMPTY_RAW_VALUE)*SCALE_KG_PER_RAW_UNIT;
    }
    return weightInKg;
}

//float RangerAsebaBridge::scaleInterpolation(int val) const
//{

//    bool negative = (SCALE_REFS[0][1] > SCALE_REFS[(SCALE_REFS_NROWS-1)][1]);

//    if ( (negative && val > SCALE_REFS[0][1]) || (!negative && val < SCALE_REFS[0][1]) )
//    {
//        return SCALE_REFS[0][0];
//    }
//    int pd = SCALE_REFS[0][0];
//    int pv = SCALE_REFS[0][1];

//    for (int nRow = 0; nRow < SCALE_REFS_NROWS; nRow++)
//    {
//        int d = SCALE_REFS[nRow][0];
//        int v = SCALE_REFS[nRow][1];

//        if ( (negative && val > v) || (!negative && val < v) )
//        {
//            float a = float(pd - d) / (pv - v);
//            float b = float(d * pv - v * pd) / (pv -v);
//            return a * val + b;
//        }
//        pd = d;
//        pv = v;
//    }

//    return SCALE_REFS[(SCALE_REFS_NROWS-1)][0];
//}

void RangerAsebaBridge::incomingData(Dashel::Stream *stream)
{
    // receive message
    Message *message = 0;
    try
    {
        // deserialize message using Aseba::Message::receive() static function
        message = Message::receive(stream);
    }
    catch (DashelException e)
    {
        // if this stream has a problem, ignore it for now,
        // and let Hub call connectionClosed later.
        ROS_ERROR_STREAM("error while receiving message: " << e.what());
        return;
    }

    // pass message to description manager, which builds
    // the node descriptions in background
    DescriptionsManager::processMessage(message);

    const UserMessage *userMessage(dynamic_cast<UserMessage *>(message));
    if (userMessage)
    {
        if (userMessage->type == RANGER_MAIN_FEEDBACK_WITH_ENCODERS_EVENT) {
            l_encoder = -userMessage->data[15];
            r_encoder = userMessage->data[13];
            is_charging = (userMessage->data[17] != 0);
            scale = scaleInterpolation(userMessage->data[7]);
            voltage = userMessage->data[5];
        }
    }

    delete message;
}

void RangerAsebaBridge::setSpeed(int l_wheel, int r_wheel) {
    vector<int> speeds;
    speeds.push_back(l_wheel);
    speeds.push_back(r_wheel);
    emit(RANGER_SET_SPEED_EVENT, speeds);
}

void RangerAsebaBridge::emit(int event, vector<int> args) {
    UserMessage::DataVector data;
    for (vector<int>::iterator it = args.begin() ; it != args.end(); ++it) {data.push_back(*it);}
    UserMessage userMessage(event, data);
    userMessage.serialize(targetStream);
    targetStream->flush();
}

void RangerAsebaBridge::emit(int event, const vector<string>& args)
{
    // build event and emit
    UserMessage::DataVector data;
    for (size_t i=2; i<args.size(); ++i)
        data.push_back(atoi(args[i].c_str()));
    UserMessage userMessage(event, data);
    userMessage.serialize(targetStream);
    targetStream->flush();
}


