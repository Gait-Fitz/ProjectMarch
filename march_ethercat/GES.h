//
// Created by Martijn on 5-2-19.
//

#ifndef PROJECT_GES_H
#define PROJECT_GES_H

#include "Slave.h"

class GES : public Slave
{
public:
  GES(std::string name, uint16 number);
  virtual void publish() = 0;
};

#endif  // PROJECT_GES_H
