#ifndef ARM_CONTROL_QUEUE_H
#define ARM_CONTROL_QUEUE_H

#include "CommonDefines.h"

/* === Trajectory Queue API === */
struct Trajectory {
  SI_8 front, rear, maxSize;
  UI_16 *qBase[MAX_JOINT];
};

class ArmControlQueue {
public:
  ArmControlQueue();
  void init(
    UI_16 queueBuffer[MAX_JOINT][TRAJECTORY_QUEUE_SIZE]
  );
  UI_8 QueueIsFull(Trajectory *queue);
  UI_8 QueueIsEmpty(Trajectory *queue);
  void QueueEndQueue(
    Trajectory *queue,
    UI_16 *data
  );
  void QueueDequeue(
    Trajectory *queue,
    UI_16 *dataOut
  );

  Trajectory trajectory;
};

#endif
