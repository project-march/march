// Copyright 2019 Project March.

struct ErrorCounter
{
  ErrorCounter() : count(0)
  {
  }

  void cb(const march_shared_resources::Error& msg)
  {
    ++count;
  }

  uint32_t count;
};