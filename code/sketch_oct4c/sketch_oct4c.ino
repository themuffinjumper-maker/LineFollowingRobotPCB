#define SENSOR_1_PIN A0
#define SENSOR_2_PIN A1
#define SENSOR_3_PIN A2
#define SENSOR_4_PIN A3
#define SENSOR_5_PIN A4

#define SENSOR_THRESHOLD 10

#define MOTOR_1_PIN 5
#define MOTOR_2_PIN 6

#define MOTOR_MAX (256 * 3 / 4) // 75%
#define MOTOR_MIN_OUTER (256 * 2 / 4) // 50%
#define MOTOR_MIN_INNER (256 * 1 / 4) // 25%

// The output from the PID may not exceed this in either direction.
#define PID_CLAMP 256
int clamp(int v, int low, int high) {
  if (v < low) return low;
  if (v > high) return high;
  return v;
}


class MotorOutput {
  public:
    int left, right;
    int const leftPin, rightPin;

    // -ve => left, +ve => right
    // This should be the output from the PIDController?
    void update(int u) {
      // Use 75-75 to go straight, and 50-25 at max turn.
      bool direction = u < 0;
      int uAbs = direction ? -u : u;
      
      // Scale both the inner and outer speeds by different amounts.
      int inner = (uAbs * (MOTOR_MAX - MOTOR_MIN_INNER) + MOTOR_MIN_INNER) / PID_CLAMP;
      int outer = (uAbs * (MOTOR_MAX - MOTOR_MIN_OUTER) + MOTOR_MIN_OUTER) / PID_CLAMP;
      if (direction) {
        left = inner;
        right = outer;
      } else {
        left = outer;
        right = inner;
      }

      analogWrite(leftPin, left);
      analogWrite(rightPin, right);
    }

    MotorOutput(int leftPinNo, int rightPinNo): leftPin(leftPinNo), rightPin(rightPinNo) {}

    void init() {
      pinMode(leftPin, OUTPUT);
      pinMode(rightPin, OUTPUT);
    }

};

// Handles the input from the sensors.
class SensorInput {
  public:

    // limitLeft, left, centre, right, limitRight
    bool activePins[5];
    int pinNumbers[5];

    SensorInput(int pins[5]): activePins{0, 0, 0, 0, 0} {
      for (int i = 0; i < 5; i++) {
        activePins[i] = pins[i];
      }
    }

    void init() {
      for (int pin: activePins) {
        pinMode(pin, INPUT);
      }
    }

    void update() {
      for (int i = 0; i < 5; i++) { 
        int analogState = analogRead(pinNumbers[i]);
        activePins[i] = analogState >= SENSOR_THRESHOLD;
      }
    }
};

class PIDController {
  private:
    int integral;
    int prevErr;

  public:
    PIDController(int initProp, int initInteg, int initDeriv): 
      kProp(initProp), kDeriv(initDeriv), kInteg(initInteg) {}

    int kProp, kInteg, kDeriv;

    // Step the controller through one time unit.
    int step(int err) {
      int uProp = -kProp * err;

      int uInteg = -kInteg * integral;
      integral += err;

      int uDeriv = -kDeriv * (err - prevErr);
      prevErr = err;

      int out = uProp + uDeriv + uInteg;
      return clamp(out, -PID_CLAMP, PID_CLAMP);
    }
};

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
    SensorStates(): currPos(NEUTRAL) {}

    LinePosition currPos;

    // Updates the line position.
    LinePosition step(SensorInput &input) {
      currPos = nextLinePosition(
        input.activePins[0],
        input.activePins[1],
        input.activePins[2],
        input.activePins[3],
        input.activePins[4]
      );
      return currPos;
    }

  private:
    LinePosition nextLinePosition(
      bool limitLeft, 
      bool left, 
      bool centre,
      bool right, 
      bool limitRight
    ) {
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
};

// SensorInput input;
MotorOutput output(5, 6);
// SensorStates state;
PIDController controller(0, 0, 0);

void setup() {
  output.init();
  
}

void loop() {
  // put your main code here, to run repeatedly:

  // TODO: Use the PID
  // input.update();
  // state.step(input);
  output.update(0);
}
