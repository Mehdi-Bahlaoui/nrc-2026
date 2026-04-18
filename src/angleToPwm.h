#pragma once
#include <Arduino.h>

const int calibAngles[7] = {0, 30, 60, 90, 120, 150, 180};

// Custom data fro team roboticore
const int team_roboticore[6][7]={ {389, 715, 968, 1428, 1827, 2212, 2528},
                        {360, 735, 1018, 1493, 1892, 2222, 2550},
                        {389, 715, 968, 1478, 1827, 2212, 2555},
                        {385, 785, 1078, 1538, 1927, 2322, 2670},
                        {360, 740, 1038, 1498, 1892, 2252, 2590},
                        {360, 710, 999, 1388, 1732, 2082, 2450} 
                    };


                    int pulseForServoForAngle(int servoIndex, float angleDeg) {
  if (angleDeg <= calibAngles[0]) return team_roboticore[servoIndex][0];
  if (angleDeg >= calibAngles[6]) return team_roboticore[servoIndex][6];

  for (int i = 0; i < 6; i++) {
    if (angleDeg >= calibAngles[i] && angleDeg <= calibAngles[i + 1]) {
      float t = (angleDeg - calibAngles[i]) / float(calibAngles[i + 1] - calibAngles[i]);
      return int(team_roboticore[servoIndex][i] + t * (team_roboticore[servoIndex][i + 1] - team_roboticore[servoIndex][i]));
    }
  }
  return team_roboticore[servoIndex][0];
}