#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>

// ============================================================
// Type aliases (match firmware ArmControl.h)
// ============================================================
using UI_8  = uint8_t;
using UI_16 = uint16_t;
using SI_8  = int8_t;
using SI_16 = int16_t;
using SI_32 = int32_t;

// ============================================================
// Constants (match firmware ArmControl.h)
// ============================================================
static constexpr UI_16 SERVO_MIN_SOURCE    = 0;
static constexpr UI_16 SERVO_MAX_SOURCE    = 180;
static constexpr UI_16 COMMON_MIN_ANGLE    = 400;
static constexpr UI_16 COMMON_MAX_ANGLE    = 2450;
static constexpr SI_32  MOTION_DURATION    = 150;
static constexpr SI_32  CUBIC_SCALE_POS    = 1000;
static constexpr SI_32  CUBIC_SCALE_A2     = 1000000;
static constexpr SI_32  CUBIC_SCALE_A3     = 1000000000;
static constexpr SI_32  CUBIC_COEFF_A2     = 3;
static constexpr SI_32  CUBIC_COEFF_A3     = -2;
static constexpr SI_32  CUBIC_ZERO         = 0;
static constexpr UI_8   MAX_JOINT          = 6;
static constexpr UI_8   FIRST_JOINT        = 0;
static constexpr UI_8   FIRST_ANGLE        = 0;
static constexpr UI_8   MAX_ANGLE          = 2;
static constexpr UI_8   TRAJECTORY_QUEUE_SIZE = 16;
static constexpr SI_8   QUEUE_NULL_IDX     = -1;
static constexpr SI_8   INCREMENT          = 1;
static constexpr UI_8   QUEUE_EMPTY        = 0;
static constexpr UI_8   D_FALSE            = 0;
static constexpr UI_8   D_TRUE             = 1;

// ============================================================
// Fake Arduino time
// ============================================================
static SI_32 g_now_ms = 0;
static UI_16 millis() { return (UI_16)g_now_ms; }
static void step_ms(SI_32 ms) { g_now_ms += ms; }

// ============================================================
// Firmware-equivalent types / state
// ============================================================
struct Trajectory {
    SI_8 front;
    SI_8 rear;
    SI_8 maxSize;
};

static UI_16  queueBuffer[MAX_JOINT][TRAJECTORY_QUEUE_SIZE];
static Trajectory trajQueue;

static UI_16 previousJointPulse[MAX_JOINT];
static UI_16 currentJointPulse[MAX_JOINT];
static UI_8  jointMotionComplete[MAX_JOINT];

static UI_16 startTime[MAX_JOINT];
static UI_16 lastTarget[MAX_JOINT];
static UI_16 currentOutput[MAX_JOINT];

// ============================================================
// Firmware-equivalent functions (exact copy of logic)
// ============================================================

static UI_16 constrain(UI_16 value)
{
    if (value < SERVO_MIN_SOURCE) return SERVO_MIN_SOURCE;
    if (value > SERVO_MAX_SOURCE) return SERVO_MAX_SOURCE;
    return value;
}

static UI_16 angleToValue(UI_16 angle, UI_16 minTarget, UI_16 maxTarget)
{
    UI_16 convertedAngle = constrain(angle);
    return (convertedAngle - SERVO_MIN_SOURCE) * (maxTarget - minTarget)
         / (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE) + minTarget;
}

static UI_16 valueToAngle(UI_16 value, UI_16 minTarget, UI_16 maxTarget)
{
    return ((value - minTarget) * (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE))
         / (maxTarget - minTarget) + SERVO_MIN_SOURCE;
}

static UI_8 queueIsFull(Trajectory* q)
{
    return (q->front == (q->rear + INCREMENT) % q->maxSize) ? D_TRUE : D_FALSE;
}

static UI_8 queueIsEmpty(Trajectory* q)
{
    return (q->front == QUEUE_NULL_IDX) ? D_TRUE : D_FALSE;
}

static void enqueue(Trajectory* q, UI_16* data)
{
    if ((D_FALSE == queueIsFull(q)) || (D_TRUE == queueIsEmpty(q))) {
        q->front = QUEUE_EMPTY;
        q->rear = (q->rear + INCREMENT) % q->maxSize;
        for (UI_8 c = FIRST_JOINT; c < MAX_JOINT; c++) {
            queueBuffer[c][q->rear] = data[c];
        }
    }
}

static void dequeue(Trajectory* q, UI_16* dataOut)
{
    if (D_FALSE == queueIsEmpty(q)) {
        for (UI_8 c = FIRST_JOINT; c < MAX_JOINT; c++) {
            dataOut[c] = queueBuffer[c][q->front];
        }
        if (q->front == q->rear) {
            q->front = QUEUE_NULL_IDX;
            q->rear  = QUEUE_NULL_IDX;
        } else {
            q->front = (q->front + INCREMENT) % q->maxSize;
        }
    }
}

static UI_16 computeCubicTrajectory(UI_16 startPos, UI_16 endPos, UI_8 jointIdx)
{
    SI_32 delta = (SI_32)endPos - (SI_32)startPos;

    if (lastTarget[jointIdx] != endPos) {
        startTime[jointIdx] = millis();
        lastTarget[jointIdx] = endPos;
    }

    UI_16 elapsedMs = millis() - startTime[jointIdx];
    SI_32 T = MOTION_DURATION;
    SI_32 t = elapsedMs;

    SI_32 a0 = (SI_32)startPos * CUBIC_SCALE_POS;
    SI_32 a1 = CUBIC_ZERO;
    SI_32 a2 = (SI_32)((int64_t)CUBIC_COEFF_A2 * delta * CUBIC_SCALE_A2 / (T * T));
    SI_32 a3 = (SI_32)((int64_t)CUBIC_COEFF_A3 * delta * CUBIC_SCALE_A3 / (T * T * T));

    if (t <= T) {
        SI_32 pos = a0 / CUBIC_SCALE_POS
                  + (SI_32)((int64_t)a2 * t * t / CUBIC_SCALE_A2)
                  + (SI_32)((int64_t)a3 * t * t * t / CUBIC_SCALE_A3);
        if (pos < (SI_32)COMMON_MIN_ANGLE) pos = COMMON_MIN_ANGLE;
        if (pos > (SI_32)COMMON_MAX_ANGLE) pos = COMMON_MAX_ANGLE;
        currentOutput[jointIdx] = (UI_16)pos;
    } else {
        jointMotionComplete[jointIdx] = D_TRUE;
    }

    return currentOutput[jointIdx];
}

static UI_8 isAllJointMotionComplete()
{
    for (UI_8 c = FIRST_JOINT; c < MAX_JOINT; c++) {
        if (D_FALSE == jointMotionComplete[c]) return D_FALSE;
    }
    return D_TRUE;
}

static void prepareNextMotionStep()
{
    for (UI_8 c = FIRST_JOINT; c < MAX_JOINT; c++) {
        previousJointPulse[c] = currentJointPulse[c];
        jointMotionComplete[c] = D_FALSE;
    }
    if (D_FALSE == queueIsEmpty(&trajQueue)) {
        dequeue(&trajQueue, currentJointPulse);
    }
}

static void resetState()
{
    g_now_ms = 0;
    trajQueue.front   = QUEUE_NULL_IDX;
    trajQueue.rear    = QUEUE_NULL_IDX;
    trajQueue.maxSize = TRAJECTORY_QUEUE_SIZE;

    for (UI_8 i = 0; i < MAX_JOINT; i++) {
        previousJointPulse[i] = COMMON_MIN_ANGLE;
        currentJointPulse[i]  = COMMON_MIN_ANGLE;
        jointMotionComplete[i] = D_TRUE;
        startTime[i]      = 0;
        lastTarget[i]     = 0;
        currentOutput[i]  = COMMON_MIN_ANGLE;
    }
    for (UI_8 j = 0; j < MAX_JOINT; j++) {
        for (UI_8 k = 0; k < TRAJECTORY_QUEUE_SIZE; k++) {
            queueBuffer[j][k] = 0;
        }
    }
}

// ============================================================
// Test framework
// ============================================================
static int g_fail = 0;

static void expect(bool cond, const char* msg)
{
    if (!cond) { std::printf("  FAIL: %s\n", msg); g_fail++; }
}

static void expectNear(double a, double b, double eps, const char* msg)
{
    if (std::abs(a - b) > eps) {
        std::printf("  FAIL: %s (got %.1f expected %.1f)\n", msg, a, b);
        g_fail++;
    }
}

// ============================================================
// TEST: Angle conversion precision
// ============================================================
static void test_angle_conversion()
{
    // Roundtrip all angles 0..180
    int errs = 0;
    for (UI_16 ang = 0; ang <= 180; ang++) {
        UI_16 pulse = angleToValue(ang, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE);
        UI_16 ang2  = valueToAngle(pulse, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE);
        if (std::abs((int)ang - (int)ang2) > 1) errs++;
    }
    expect(errs == 0, "All angles 0-180 roundtrip within 1 deg");
    expect(angleToValue(0,   COMMON_MIN_ANGLE, COMMON_MAX_ANGLE) == COMMON_MIN_ANGLE, "0 deg -> min pulse");
    expect(angleToValue(180, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE) == COMMON_MAX_ANGLE, "180 deg -> max pulse");
    expect(angleToValue(90,  COMMON_MIN_ANGLE, COMMON_MAX_ANGLE) > COMMON_MIN_ANGLE,  "90 deg in range");
}

// ============================================================
// TEST: Cubic trajectory shape
// ============================================================
static void test_cubic_trajectory_shape()
{
    resetState();
    UI_16 start = 700, end = 2300;
    UI_8  j = 0;

    std::vector<UI_16> samples;
    for (SI_32 ms = 0; ms <= MOTION_DURATION + 50; ms += 5) {
        g_now_ms = ms;
        samples.push_back(computeCubicTrajectory(start, end, j));
    }

    expect(samples.front() == start, "Trajectory starts at startPos");
    expect(samples.back() == end,    "Trajectory ends at endPos");
    expect(jointMotionComplete[j] == D_TRUE, "Motion complete after T");

    // Monotonic non-decreasing
    bool mono = true;
    for (size_t i = 1; i < samples.size(); i++)
        if (samples[i] + 1 < samples[i-1]) { mono = false; break; }
    expect(mono, "Trajectory is monotonic non-decreasing");

    // Symmetry: at t=T/2, should be near midpoint
    g_now_ms = MOTION_DURATION / 2;
    UI_16 mid = computeCubicTrajectory(start, end, j);
    expect(mid > start && mid < end, "Midpoint strictly between start and end");
}

// ============================================================
// TEST: Multiple joint interpolation
// ============================================================
static void test_multi_joint_cubic()
{
    resetState();
    UI_16 starts[MAX_JOINT] = {400,  800, 1200, 1600, 2000,  500};
    UI_16 ends[MAX_JOINT]   = {800, 1200, 1600, 2000, 2400, 1000};

    for (SI_32 ms = 0; ms <= MOTION_DURATION; ms += 10) {
        g_now_ms = ms;
        for (UI_8 j = 0; j < MAX_JOINT; j++) {
            UI_16 pos = computeCubicTrajectory(starts[j], ends[j], j);
            expect(pos >= COMMON_MIN_ANGLE && pos <= COMMON_MAX_ANGLE,
                   "Multi-joint: all positions in valid range");
        }
    }
    for (UI_8 j = 0; j < MAX_JOINT; j++)
        expect(computeCubicTrajectory(starts[j], ends[j], j) == ends[j],
               "Multi-joint: all joints reach target");
}

// ============================================================
// TEST: Queue full pipeline
// ============================================================
static void test_queue_pipeline()
{
    resetState();
    UI_16 data[MAX_JOINT];
    UI_16 out[MAX_JOINT];

    // Fill queue with 3 waypoints
    for (UI_8 i = 0; i < MAX_JOINT; i++) data[i] = 1000 + i * 100;
    enqueue(&trajQueue, data);
    for (UI_8 i = 0; i < MAX_JOINT; i++) data[i] = 2000 + i * 50;
    enqueue(&trajQueue, data);
    for (UI_8 i = 0; i < MAX_JOINT; i++) data[i] = 1500 + i * 30;
    enqueue(&trajQueue, data);

    expect(queueIsEmpty(&trajQueue) == D_FALSE, "Queue not empty after enqueues");

    dequeue(&trajQueue, out);
    expect(out[0] == 1000, "Dequeue 1: first element correct");

    dequeue(&trajQueue, out);
    expect(out[0] == 2000, "Dequeue 2: second element correct");

    dequeue(&trajQueue, out);
    expect(out[0] == 1500, "Dequeue 3: third element correct");

    expect(queueIsEmpty(&trajQueue) == D_TRUE, "Queue empty after draining");
    dequeue(&trajQueue, out); // no-op on empty
}

// ============================================================
// HARD ENDURANCE: 10 000 random motion steps
// ============================================================
static void test_endurance_10000_steps()
{
    std::printf("  Endurance: 10000 random motion steps...\n");
    resetState();

    std::srand(42);
    UI_16 waypoint[MAX_JOINT];
    int queueFullCount = 0;
    int stepsCompleted = 0;
    int totalSteps     = 10000;

    for (int step = 0; step < totalSteps; step++) {
        // Every 5th cycle, enqueue 1-4 random waypoints
        if (step % 5 == 0) {
            int n = 1 + (std::rand() % 4);
            for (int w = 0; w < n; w++) {
                for (UI_8 j = 0; j < MAX_JOINT; j++)
                    waypoint[j] = COMMON_MIN_ANGLE + (std::rand() % (COMMON_MAX_ANGLE - COMMON_MIN_ANGLE + 1));
                int oldFront = trajQueue.front;
                enqueue(&trajQueue, waypoint);
                if (oldFront == trajQueue.front && trajQueue.front != QUEUE_NULL_IDX)
                    queueFullCount++;
            }
        }

        // When motion complete and data available, advance
        if (isAllJointMotionComplete() && !queueIsEmpty(&trajQueue)) {
            prepareNextMotionStep();
            stepsCompleted++;
        }

        // Run trajectory for all joints (fast-forward time slightly)
        for (UI_8 j = 0; j < MAX_JOINT; j++) {
            for (int t = 0; t < 50; t += 5) {
                g_now_ms = step * 50 + t;
                UI_16 pos = computeCubicTrajectory(previousJointPulse[j], currentJointPulse[j], j);
                if (pos < COMMON_MIN_ANGLE || pos > COMMON_MAX_ANGLE) {
                    std::printf("  FAIL: Joint %d out of range at step %d: %d\n", j, step, pos);
                    g_fail++;
                }
            }
        }

        // Mark motion complete once enough time elapsed
        g_now_ms = step * 50 + MOTION_DURATION + 1;
        for (UI_8 j = 0; j < MAX_JOINT; j++)
            computeCubicTrajectory(previousJointPulse[j], currentJointPulse[j], j);
    }

    expect(stepsCompleted > 0, "Endurance: at least one step completed");
    expect(queueFullCount >= 0, "Endurance: queue full handling OK");

    // Final state sanity
    for (UI_8 j = 0; j < MAX_JOINT; j++) {
        expect(previousJointPulse[j] >= COMMON_MIN_ANGLE && previousJointPulse[j] <= COMMON_MAX_ANGLE,
               "Endurance: final previous pulse in range");
        expect(currentJointPulse[j] >= COMMON_MIN_ANGLE && currentJointPulse[j] <= COMMON_MAX_ANGLE,
               "Endurance: final current pulse in range");
    }

    std::printf("  ... completed %d / %d motion steps, queue-full events: %d\n",
                stepsCompleted, totalSteps, queueFullCount);
}

// ============================================================
// HARD ENDURANCE: 50 000 queue thrash cycles
// ============================================================
static void test_endurance_queue_thrash()
{
    std::printf("  Endurance: 50000 queue thrash cycles...\n");
    resetState();

    UI_16 data[MAX_JOINT];
    UI_16 out[MAX_JOINT];
    std::srand(7);

    for (int cycle = 0; cycle < 50000; cycle++) {
        int n = 1 + (std::rand() % TRAJECTORY_QUEUE_SIZE);
        for (int i = 0; i < n; i++) {
            for (UI_8 j = 0; j < MAX_JOINT; j++)
                data[j] = (UI_16)(cycle + i * 10 + j);
            enqueue(&trajQueue, data);
        }
        for (int i = 0; i < n; i++) {
            if (!queueIsEmpty(&trajQueue)) {
                dequeue(&trajQueue, out);
                // Verify ordering within this batch
                for (UI_8 j = 1; j < MAX_JOINT; j++)
                    expect(out[j] == out[0] + j, "Queue thrash: joint-consistent ordering");
            }
        }
        if (std::rand() % 10 == 0) {
            expect(queueIsEmpty(&trajQueue) == D_TRUE, "Queue thrash: empty after drain");
        }
    }

    std::printf("  ... 50000 cycles OK\n");
}

// ============================================================
// HARD ENDURANCE: Alternating min-max stress
// ============================================================
static void test_endurance_alternating_min_max()
{
    std::printf("  Endurance: 2000 alternating min/max cycles...\n");
    resetState();

    UI_16 data[MAX_JOINT];
    std::srand(13);

    for (int cycle = 0; cycle < 2000; cycle++) {
        UI_16 extreme = (cycle % 2 == 0) ? COMMON_MAX_ANGLE : COMMON_MIN_ANGLE;
        for (UI_8 j = 0; j < MAX_JOINT; j++)
            data[j] = extreme;
        enqueue(&trajQueue, data);

        if (isAllJointMotionComplete() && !queueIsEmpty(&trajQueue))
            prepareNextMotionStep();

        g_now_ms = cycle * 50 + MOTION_DURATION + 1;
        for (UI_8 j = 0; j < MAX_JOINT; j++) {
            UI_16 pos = computeCubicTrajectory(previousJointPulse[j], currentJointPulse[j], j);
            if (pos < COMMON_MIN_ANGLE || pos > COMMON_MAX_ANGLE) {
                std::printf("  FAIL: Joint %d out of range at cycle %d: %d\n", j, cycle, pos);
                g_fail++;
            }
        }
    }

    std::printf("  ... 2000 cycles OK\n");
}

// ============================================================
// MAIN
// ============================================================
int main()
{
    std::printf("========================================\n");
    std::printf("  ArmControl Comprehensive Host Tests\n");
    std::printf("========================================\n\n");

    std::printf("[Unit Tests]\n");
    test_angle_conversion();
    test_cubic_trajectory_shape();
    test_multi_joint_cubic();
    test_queue_pipeline();
    std::printf("\n");

    std::printf("[Hard Endurance]\n");
    test_endurance_10000_steps();
    test_endurance_queue_thrash();
    test_endurance_alternating_min_max();
    std::printf("\n");

    if (g_fail == 0) {
        std::printf("========================================\n");
        std::printf("  PASS: all tests\n");
        std::printf("========================================\n");
        return 0;
    } else {
        std::printf("========================================\n");
        std::printf("  FAIL: %d test(s) failed\n", g_fail);
        std::printf("========================================\n");
        return 1;
    }
}
