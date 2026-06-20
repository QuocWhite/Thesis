#include <cstdint>
#include <cstdio>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

// -----------------------------
// Minimal typedefs (match yours)
// -----------------------------
using UI_8  = uint8_t;
using UI_16 = uint16_t;
using SI_32 = int32_t;

// -----------------------------
// Constants (tune to your project)
// -----------------------------
static constexpr UI_16 SERVO_MIN_SOURCE = 0;
static constexpr UI_16 SERVO_MAX_SOURCE = 180;

// Example servo pulse targets (microseconds)
static constexpr UI_16 COMMON_MIN_ANGLE = 500;
static constexpr UI_16 COMMON_MAX_ANGLE = 2500;

// Cubic trajectory constants (match your defines if different)
static constexpr SI_32 MOTION_DURATION = 1000; // ms
static constexpr SI_32 CUBIC_SCALE_POS = 1000000000; // 1e9
static constexpr SI_32 CUBIC_SCALE_A2  = 1000000000;
static constexpr SI_32 CUBIC_SCALE_A3  = 1000000000;
static constexpr SI_32 CUBIC_ZERO      = 0;
static constexpr SI_32 CUBIC_COEFF_A2  = 3;   // typical 3*delta/T^2
static constexpr SI_32 CUBIC_COEFF_A3  = -2;  // typical -2*delta/T^3
static constexpr SI_32 CUBIC_SCALE_A2_CONST = 1000000000;
static constexpr SI_32 CUBIC_SCALE_A3_CONST = 1000000000;

static constexpr UI_8 MAX_JOINT = 6;
static constexpr UI_8 FIRST_JOINT = 0;

// -----------------------------
// Fake Arduino time
// -----------------------------
static SI_32 g_now_ms = 0;
static UI_16 millis() { return (UI_16)g_now_ms; } // match your UI_16 usage
static void step_ms(SI_32 ms) { g_now_ms += ms; }

// -----------------------------
// Logic copied / adapted from your code
// (pure-ish parts for IT tests)
// -----------------------------
static UI_16 ConsTrain(UI_16 value)
{
  if (value < SERVO_MIN_SOURCE) return SERVO_MIN_SOURCE;
  if (value > SERVO_MAX_SOURCE) return SERVO_MAX_SOURCE;
  return value;
}

static UI_16 AngletoValue(UI_16 angle, UI_16 minTarget, UI_16 maxTarget)
{
  UI_16 convertedAngle = ConsTrain(angle);
  UI_16 value = (UI_16)((convertedAngle - SERVO_MIN_SOURCE) * (maxTarget - minTarget)
              / (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE) + minTarget);
  return value;
}

static UI_16 ValuetoAngle(UI_16 value, UI_16 minTarget, UI_16 maxTarget)
{
  UI_16 angle = (UI_16)(((value - minTarget) * (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE))
              / (maxTarget - minTarget) + SERVO_MIN_SOURCE);
  return angle;
}

// Motion complete flags (like your global/arrays)
static UI_8 jointMotionComplete[MAX_JOINT];

// Your cubic trajectory function, using our fake millis()
static UI_16 computeCubicTrajectory(UI_16 startPos, UI_16 endPos, UI_8 jointIdx)
{
  static UI_16 startTime[MAX_JOINT]   = {0};
  static UI_16 lastTarget[MAX_JOINT]  = {0};
  static UI_16 currentOutput[MAX_JOINT] = {0};

  SI_32 a0, a1, a2, a3;
  UI_16 elapsedMs;
  SI_32 T, t, delta, pos;

  delta = (SI_32)endPos - (SI_32)startPos;

  if (lastTarget[jointIdx] != endPos) {
    startTime[jointIdx] = millis();
    lastTarget[jointIdx] = endPos;
    elapsedMs = 0;
    jointMotionComplete[jointIdx] = 0;
  }

  elapsedMs = (UI_16)(millis() - startTime[jointIdx]);
  T = MOTION_DURATION;
  t = elapsedMs;

  // scaled coeffs (adapted to your style)
  a0 = (SI_32)startPos * CUBIC_SCALE_POS;
  a1 = 0;
  a2 = (CUBIC_COEFF_A2 * delta * CUBIC_SCALE_A2_CONST) / (T * T);
  a3 = (CUBIC_COEFF_A3 * delta * CUBIC_SCALE_A3_CONST) / (T * T * T);

  if (t <= T) {
    pos = a0 / CUBIC_SCALE_POS
        + (a1 * t)
        + (a2 * t * t) / CUBIC_SCALE_A2
        + (a3 * t * t * t) / CUBIC_SCALE_A3;

    // clamp to valid pulse range (optional but common for safety)
    pos = std::clamp(pos, (SI_32)COMMON_MIN_ANGLE, (SI_32)COMMON_MAX_ANGLE);

    currentOutput[jointIdx] = (UI_16)pos;
  } else {
    jointMotionComplete[jointIdx] = 1;
    currentOutput[jointIdx] = endPos; // usually desired at end
  }

  return currentOutput[jointIdx];
}

// -----------------------------
// Tiny test framework (no dependencies)
// -----------------------------
static int g_fail = 0;

static void expect_true(bool cond, const char* msg)
{
  if (!cond) {
    std::printf("FAIL: %s\n", msg);
    g_fail++;
  }
}

static void expect_near(double a, double b, double eps, const char* msg)
{
  if (std::fabs(a - b) > eps) {
    std::printf("FAIL: %s (got %.6f expected %.6f)\n", msg, a, b);
    g_fail++;
  }
}

// -----------------------------
// Tests
// -----------------------------
static void test_angle_value_roundtrip()
{
  // round-trip: angle -> pulse -> angle (should be close)
  for (UI_16 ang : {0, 10, 45, 90, 135, 180}) {
    UI_16 pulse = AngletoValue(ang, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE);
    UI_16 ang2  = ValuetoAngle(pulse, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE);
    expect_near((double)ang, (double)ang2, 2.0, "Angle->Value->Angle roundtrip");
  }

  // constrain behavior
  UI_16 pulse_low = AngletoValue(0, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE);
  UI_16 pulse_hi  = AngletoValue(180, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE);
  expect_true(pulse_low == COMMON_MIN_ANGLE, "Angle 0 maps to minTarget");
  expect_true(pulse_hi  == COMMON_MAX_ANGLE, "Angle 180 maps to maxTarget");
}

static void test_cubic_trajectory_shape_and_completion()
{
  // reset time + flags
  g_now_ms = 0;
  for (UI_8 j=0;j<MAX_JOINT;j++) jointMotionComplete[j]=0;

  UI_8 joint = 0;
  UI_16 start = 700;
  UI_16 end   = 2300;

  std::vector<UI_16> y;
  std::vector<SI_32> t;

  // sample from 0..T+200ms
  for (SI_32 ms = 0; ms <= (SI_32)MOTION_DURATION + 200; ms += 20) {
    g_now_ms = ms;
    UI_16 out = computeCubicTrajectory(start, end, joint);
    y.push_back(out);
    t.push_back(ms);
  }

  // basic checks
  expect_true(y.front() >= start - 5 && y.front() <= start + 50, "Trajectory starts near startPos");
  expect_true(y.back() == end, "After T, output equals endPos");
  expect_true(jointMotionComplete[joint] == 1, "Motion marked complete after T");

  // monotonic (for increasing move); allow tiny numerical noise
  bool nondecreasing = true;
  for (size_t i=1;i<y.size();i++) {
    if (y[i] + 2 < y[i-1]) { nondecreasing = false; break; }
  }
  expect_true(nondecreasing, "Trajectory is (mostly) nondecreasing for increasing command");

  // write CSV so you can plot it
  std::FILE* f = std::fopen("trajectory_log.csv", "wb");
  if (f) {
    std::fprintf(f, "t_ms,pulse\n");
    for (size_t i=0;i<y.size();i++) {
      std::fprintf(f, "%u,%u\n", (unsigned)t[i], (unsigned)y[i]);
    }
    std::fclose(f);
    std::printf("Wrote trajectory_log.csv\n");
  } else {
    std::printf("WARN: could not write trajectory_log.csv\n");
  }
}

int main()
{
  std::printf("Running ArmControl host IT tests...\n");

  test_angle_value_roundtrip();
  test_cubic_trajectory_shape_and_completion();

  if (g_fail == 0) {
    std::printf("PASS: all tests\n");
    return 0;
  } else {
    std::printf("FAIL: %d test(s)\n", g_fail);
    return 1;
  }
}
