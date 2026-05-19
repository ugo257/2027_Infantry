#ifndef PUBSUB_H
#define PUBSUB_H

#include "stdint.h"

#define MAX_TOPIC_NAME_LEN 32
#define MAX_TOPIC_COUNT 12
#define QUEUE_SIZE 1

typedef struct mqt
{
    void *queue[QUEUE_SIZE];
    uint16_t data_len;
    uint8_t front_idx;
    uint8_t back_idx;
    uint8_t temp_size;
    struct mqt *next_subs_queue;
} Subscriber_t;

typedef struct ent
{
    char topic_name[MAX_TOPIC_NAME_LEN + 1];
    uint16_t data_len;
    Subscriber_t *first_subs;
    struct ent *next_topic_node;
    uint8_t pub_registered_flag;
} Publisher_t;

Subscriber_t *SubRegister(char *name, uint16_t data_len);
Publisher_t *PubRegister(char *name, uint16_t data_len);
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr);
uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr);

#endif // !PUBSUB_H
