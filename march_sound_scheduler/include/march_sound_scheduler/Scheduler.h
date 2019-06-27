// Copyright 2019 Project March.
#ifndef MARCH_SOUND_SCHEDULER_SCHEDULER_H
#define MARCH_SOUND_SCHEDULER_SCHEDULER_H

#include <queue>

#include <ros/duration.h>
#include <ros/ros.h>

#include <sound_play/sound_play.h>

#include <march_sound_scheduler/ScheduledSound.h>

class Scheduler {

public:

    std::priority_queue<ScheduledSound, std::vector<ScheduledSound>> soundQueue;
    sound_play::SoundClient sc;

    Scheduler();

    void schedule(ScheduledSound sound);

    void spin();

    void play(ScheduledSound sound);
};

#endif  // MARCH_SOUND_SCHEDULER_SCHEDULER_H
