//
// Created by ishadijcks on 27-11-18.
//

#ifndef PROJECT_LAUNCH_API_H
#define PROJECT_LAUNCH_API_H

#include "march_api/Trigger.h"


class LaunchAPI {

public:
    static bool urdf_validator(march_api::Trigger::Request &request, march_api::Trigger::Response &response);

    static bool config_validator(march_api::Trigger::Request &request, march_api::Trigger::Response &response);

    static bool xml_validator(march_api::Trigger::Request &request, march_api::Trigger::Response &response);

};


#endif //PROJECT_LAUNCH_API_H
