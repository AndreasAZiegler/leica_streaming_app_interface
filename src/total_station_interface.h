#pragma once

#include <string>

class TSInterface {
 public:
  virtual void start(void) = 0; 
  virtual void end(void) = 0;
};
