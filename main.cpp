/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "TigerVision.h"


int main(int argc, char* argv[]) {
  TigerVision visionProcessor(320,240);
  visionProcessor.InitCamera(0);
  visionProcessor.FindTarget();
}
