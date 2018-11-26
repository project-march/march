// Copyright 2018 Project March.

#ifndef PROJECT_RESULT_H
#define PROJECT_RESULT_H


#include <string>

struct Result {
    Result(bool success, std::string message){
        this->success = success;
        this->message = message;
    }

    bool success;
    std::string message;
};


#endif //PROJECT_RESULT_H
