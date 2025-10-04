#define SENSOR_1_PIN A0
#define SENSOR_2_PIN A1
#define SENSOR_3_PIN A2
#define SENSOR_4_PIN A3
#define SENSOR_5_PIN A4

#define MOTOR_1_PIN 5
#define MOTOR_2_PIN 6

void setup() {
  // put your setup code here, to run once:
  pinMode(SENSOR_1_PIN, INPUT);
  pinMode(SENSOR_2_PIN, INPUT);
  pinMode(SENSOR_3_PIN, INPUT);
  pinMode(SENSOR_4_PIN, INPUT);
  pinMode(SENSOR_5_PIN, INPUT);

  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_2_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}

class PIDController {
  uint64_t integral;
  uint64_t prevErr;

  public:
    uint64_t kProp;
    uint64_t kInteg;
    uint64_t kDeriv;

    // Step the controller through one time unit.
    uint64_t step(uint64_t err) {
      uint64_t uProp = -kProp * err;

      uint64_t uInteg = -kInteg * integral;
      integral += err;

      uint64_t uDeriv = -kDeriv * (err - prevErr);
      prevErr = err;

      return uProp + uDeriv + uInteg;
    }
}

typedef enum {
  // Only the middle sensor is active.
  NEUTRAL = 0,
  // The middle and right sensors are active.
  NEAR_RIGHT,
  // Only the right sensor is active.
  MID_RIGHT,
  // The right sensor was the previous active sensor.
  FAR_RIGHT,
  // The right limit sensor has been tripped.
  LIMIT_RIGHT,
  // FAULT_RIGHT,
  // FAULT_LEFT = -5,
  LIMIT_LEFT = -4,
  FAR_LEFT,
  MID_LEFT,
  NEAR_LEFT
} LinePosition;

class SensorStates {
  public:
    bool limitLeft;
    bool left;
    bool centre;
    bool right;
    bool limitRight;

    LinePosition currPos;
    LinePosition step() {
      currPos = nextLinePosition();
      return currPos;
    }

  private:
    LinePosition nextLinePosition() {
      // I'm not proud of this.
      switch (currPos) {
        case NEUTRAL:
          if (left && !right) return NEAR_LEFT;
          if (right && !left) return NEAR_RIGHT;
          break;
          
        case NEAR_RIGHT:
          if ((centre || left) && !right) return NEUTRAL;
          if (!(centre || left)) return MID_RIGHT;
          break;
        case MID_RIGHT:
          if (!(centre || left || right)) return FAR_RIGHT;
          if (!right) return NEAR_RIGHT;
          break;
        case FAR_RIGHT:
          if (centre || left || right) return MID_RIGHT;
          if (limitRight) return LIMIT_RIGHT;
          break;
        case LIMIT_RIGHT:
          if (centre || left || right) return MID_RIGHT;
          break;

        case NEAR_LEFT:
          if ((centre || right) && !left) return NEUTRAL;
          if (!(centre || right)) return MID_LEFT;
          break;
        case MID_LEFT:
          if (!(centre || right || left)) return FAR_LEFT;
          if (!left) return NEAR_LEFT;
          break;
        case FAR_LEFT:
          if (centre || right || left) return MID_LEFT;
          if (limitLeft) return LIMIT_LEFT;
          break;
        case LIMIT_LEFT:
          if (centre || right || left) return MID_LEFT;
          break;
      }
    }
}