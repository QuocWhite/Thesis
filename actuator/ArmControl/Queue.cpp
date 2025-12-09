#include "Queue.h"

ArmControlQueue::ArmControlQueue() {}

void ArmControlQueue::init(
  UI_16 queueBuffer[MAX_JOINT][TRAJECTORY_QUEUE_SIZE]
) {
  trajectory.front = QUEUE_ERROR;
  trajectory.rear = QUEUE_ERROR;
  trajectory.maxSize = TRAJECTORY_QUEUE_SIZE;
  for (UI_8 i = FIRST_JOINT; i < MAX_JOINT; ++i) {
    trajectory.qBase[i] = queueBuffer[i];
  }
}
/// --- End of init() ---

UI_8 ArmControlQueue::QueueIsFull(
  Trajectory *queue
) {
  return (queue->front ==
    (queue->rear + INCREASE) % queue->maxSize) ?
    D_TRUE : D_FALSE;
}
/// --- End of QueueIsFull() ---

UI_8 ArmControlQueue::QueueIsEmpty(
  Trajectory *queue
) {
  return (queue->front == QUEUE_ERROR) ?
    D_TRUE : D_FALSE;
}
/// --- End of QueueIsEmpty() ---

void ArmControlQueue::QueueEndQueue(
  Trajectory *queue,
  UI_16 *data
) {
  if ((D_FALSE == QueueIsFull(queue)) ||
      (D_TRUE == QueueIsEmpty(queue))) {
    queue->front = QUEUE_EMPTY;
    queue->rear = (queue->rear + INCREASE) %
      queue->maxSize;
    for (UI_8 count = FIRST_JOINT;
         count < MAX_JOINT;
         count++)
      queue->qBase[count][queue->rear] = data[count];
  }
}
/// --- End of QueueEndQueue() ---

void ArmControlQueue::QueueDequeue(
  Trajectory *queue,
  UI_16 *dataOut
) {
  if (D_FALSE == QueueIsEmpty(queue)) {
    for (UI_8 count = FIRST_JOINT;
         count < MAX_JOINT;
         count++)
      dataOut[count] = queue->qBase[count][queue->front];
    if (queue->front == queue->rear) {
      queue->front = QUEUE_ERROR;
      queue->rear = QUEUE_ERROR;
    } else {
      queue->front = (queue->front + INCREASE) %
        queue->maxSize;
    }
  }
}
/// --- End of QueueDequeue() ---
