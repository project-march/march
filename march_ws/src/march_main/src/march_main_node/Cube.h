// Copyright 2018 Project March.

#ifndef MARCH_WS_SRC_MARCH_MAIN_SRC_MARCH_MAIN_NODE_CUBE_H_
#define MARCH_WS_SRC_MARCH_MAIN_SRC_MARCH_MAIN_NODE_CUBE_H_

class TestCube {
  float volume;

 public:
  explicit TestCube(float volume);

  float getVolume() const;
};

#endif  // MARCH_WS_SRC_MARCH_MAIN_SRC_MARCH_MAIN_NODE_CUBE_H_
