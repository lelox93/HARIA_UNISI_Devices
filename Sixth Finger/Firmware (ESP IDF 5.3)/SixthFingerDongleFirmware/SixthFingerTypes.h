#pragma once

enum class SixthFingerStatus {
  CLOSED,
  OPEN
};

enum class MotorStatus {
  GRASP,
  CLOSING,
  OPEN,
  HALF_CLOSED, // CLOSED SENZA GRASP
  OPENING
};

struct MotorData {
  int load = 0;
  int position = 0;
  int positionPercentage = 0;
  MotorStatus status = MotorStatus::OPEN;
};

struct SixthFingerData {
  MotorData motor1;
  SixthFingerStatus status = SixthFingerStatus::OPEN;
};

struct SixthFingerCMD {
  char cmd = 'O';
  char aux = 'O';
};

struct SixthFingerPos {
  int motor1_pos;
};
