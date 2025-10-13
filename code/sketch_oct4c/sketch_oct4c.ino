
#define SENSOR_THRESHOLD 10

#define MOTOR_MAX (256 * 3 / 4) // 75%
#define MOTOR_MIN_OUTER (MOTOR_MAX * 2 / 3) // 50%
#define MOTOR_MIN_INNER (MOTOR_MAX * 1 / 3) // 25%

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
      int inner = MOTOR_MAX - (MOTOR_MIN_INNER * uAbs) / PID_CLAMP;
      int outer = MOTOR_MAX - (MOTOR_MIN_OUTER * uAbs) / PID_CLAMP;
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
      kProp(initProp), kDeriv(initDeriv), kInteg(initInteg), integral(0), prevErr(0) {}

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
  FAR_LEFT = -2,
  LEFT,
  NEUTRAL,
  RIGHT,
  FAR_RIGHT,
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
      switch (currPos) {
        case FAR_LEFT:
          if (limitLeft) break;
          if (left) return LEFT;
          break;
        case LEFT:
          if (left) break;
          if (limitLeft) return FAR_LEFT;
          if (centre) return NEUTRAL;
          break;
        case NEUTRAL:
          if (centre) break;
          if (left) return LEFT;
          if (right) return RIGHT;
          break;
        case RIGHT:
          if (right) break;
          if (centre) return NEUTRAL;
          if (limitRight) return FAR_RIGHT;
          break;
        case FAR_RIGHT:
          if (limitRight) break;
          if (right) return RIGHT;
          break;
      }
      return currPos;
    }
};

const int INPUT_PINS[5] = {A4, A3, A2, A1, A0};

SensorInput input(INPUT_PINS);
MotorOutput output(5, 6);
SensorStates state;
PIDController controller(0, 0, 0);

void setup() {
  output.init();
  input.init();
  state = SensorStates();
}

void loop() {
  // put your main code here, to run repeatedly:

  // TODO: Use the PID
  input.update();
  state.step(input);
  output.update(0);
}
