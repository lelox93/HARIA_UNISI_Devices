#ifndef COGRIPPER_CONTROLLER_H
#define COGRIPPER_CONTROLLER_H

#include <Dynamixel2Arduino.h>
#include <HardwareSerial.h>
#include "TM2011.h"           ///< Sostituisce SLLB120100.h per la gestione dei pulsanti
#include "Motor.h"

/**
 * @brief Enumeration for the gripper status.
 */
enum class CoGripperStatus {
  CLOSED, ///< Gripper is closed.
  OPEN    ///< Gripper is open.
};

/**
 * @brief Structure that holds updated data for the gripper controller and its motors.
 */
struct CoGripperData {
  MotorData motor1;                    ///< Data specific to motor 1.
  MotorData motor2;                    ///< Data specific to motor 2.
  CoGripperStatus status = CoGripperStatus::OPEN;  ///< Current gripper status.
};

/**
 * @brief Class for controlling the CoGripper device.
 *
 * This class provides functionalities for initializing the motors,
 * calibrating limits, handling button events, controlling the gripper, and updating the device status.
 */
class CoGripperController {
public:
  /**
   * @brief Constructor for the CoGripperController.
   * @param txPin TX pin for serial communication.
   * @param rxPin RX pin for serial communication.
   * @param dirPin Direction pin used for Dynamixel control.
   */
  CoGripperController(uint8_t txPin, uint8_t rxPin, uint8_t dirPin);

/**
 * @brief Initializes the gripper controller.
 *
 * This method initializes both motors and configures the power controller.
 * For the motor model "Dynamixel MX-28 2.0", it attempts to set 12V at 3A.
 * If this configuration is not achieved (either due to failure or a fallback),
 * the method returns false.
 *
 * @return true if initialization was successful, false otherwise.
 */
bool initialize();


  /**
   * @brief Calibrates the movement limits for both motors.
   *
   * This method runs the calibration routine on both motors.
   */
  void calibrateLimits();

  /**
   * @brief Main control loop for the gripper.
   *
   * Updates motor positions, loads, statuses, and commands the motors to reach the goal positions.
   */
  void controlLoop();

  /**
   * @brief Handles button events to control the gripper.
   * @param currentState The current state of the button.
   */
  void handleButtonStates(ButtonState currentState);

  /**
   * @brief Handles torque limit adjustments using an enhanced algorithm.
   */
  void handleTorqueLimitEnhanced();

  /**
   * @brief Updates the internal data structure with the latest motor data.
   */
  void updateData();

  /**
   * @brief Retrieves the current data for the gripper controller.
   * @return A CoGripperData structure containing updated motor data and status.
   */
  CoGripperData getData();

  /**
   * @brief Sets the maximum PWM value for both motors.
   * @param pwmGoal The PWM goal value.
   */
  void setGripperMaxPWM(int pwmGoal);

  /**
   * @brief Sets the goal position for motor 1.
   * @param position The desired position.
   */
  void setGoalPositionMotor1(int position);

  /**
   * @brief Sets the goal position for motor 2.
   * @param position The desired position.
   */
  void setGoalPositionMotor2(int position);

  /**
   * @brief Commands both motors to move to the maximum position (full open).
   */
  void openAllMotors();

  /**
   * @brief Commands both motors to move to the minimum position (full closed).
   */
  void closeAllMotors();

  /**
   * @brief Stops both motors, maintaining their current positions.
   */
  void stopAllMotors();

  /**
   * @brief Converts a motor status to a descriptive string.
   * @param status The motor status.
   * @return A constant C-string describing the motor status.
   */
  const char* motorStatusToString(MotorStatus status) const;

  /**
   * @brief Prints the current gripper status to the serial monitor.
   */
  void printGripperStatus();

  /**
   * @brief Prints the current motor data of the gripper to the serial monitor.
   */
  void printGripperData();

  /**
   * @brief Gets the minimum calibrated end position for motor 1.
   * @return The minimum position.
   */
  int getMotor1EndPositionMin() const { return motor1.getEndPositionMin(); }

  /**
   * @brief Gets the maximum calibrated end position for motor 1.
   * @return The maximum position.
   */
  int getMotor1EndPositionMax() const { return motor1.getEndPositionMax(); }

  /**
   * @brief Gets the minimum calibrated end position for motor 2.
   * @return The minimum position.
   */
  int getMotor2EndPositionMin() const { return motor2.getEndPositionMin(); }

  /**
   * @brief Gets the maximum calibrated end position for motor 2.
   * @return The maximum position.
   */
  int getMotor2EndPositionMax() const { return motor2.getEndPositionMax(); }

  Motor motor1; ///< Instance of motor 1.
  Motor motor2; ///< Instance of motor 2.

private:
  HardwareSerial dynaSerial; ///< Serial interface for Dynamixel communication.
  Dynamixel2Arduino dxl;       ///< Dynamixel controller interface.
  CoGripperStatus status;      ///< Current gripper status.
  CoGripperData data;          ///< Updated gripper data.
};

#endif  // COGRIPPER_CONTROLLER_H
