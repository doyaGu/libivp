#include "ivp_test.h"

#include <ivp/collision/ivp_time_event_queue.h>

static ivp_object_handle_t make_handle(uint32_t slot, uint32_t generation)
{
    ivp_object_handle_t h;
    h.slot = slot;
    h.generation = generation;
    return h;
}

IVP_TEST(time_event_queue_orders_by_time)
{
    ivp_time_event_queue_t queue;
    ivp_time_event_queue_init(&queue, NULL);

    ivp_time_event_t e1;
    ivp_time_event_t e2;
    ivp_time_event_t e3;

    e1.trigger_time = 3.0;
    e1.obj_a = make_handle(1u, 1u);
    e1.obj_b = make_handle(2u, 1u);
    e1.type = IVP_TIME_EVENT_MINDIST_IMPACT;

    e2 = e1;
    e2.trigger_time = 1.5;
    e2.obj_a = make_handle(3u, 1u);
    e2.obj_b = make_handle(4u, 1u);

    e3 = e1;
    e3.trigger_time = 2.5;
    e3.obj_a = make_handle(5u, 1u);
    e3.obj_b = make_handle(6u, 1u);

    IVP_ASSERT_TRUE(ivp_time_event_queue_push(&queue, &e1));
    IVP_ASSERT_TRUE(ivp_time_event_queue_push(&queue, &e2));
    IVP_ASSERT_TRUE(ivp_time_event_queue_push(&queue, &e3));

    ivp_time_event_t out;
    IVP_ASSERT_TRUE(ivp_time_event_queue_pop_due(&queue, 2.0, &out));
    IVP_ASSERT_NEAR(out.trigger_time, 1.5, 1e-9);

    IVP_ASSERT_FALSE(ivp_time_event_queue_pop_due(&queue, 2.0, &out));

    IVP_ASSERT_TRUE(ivp_time_event_queue_pop_due(&queue, 3.0, &out));
    IVP_ASSERT_NEAR(out.trigger_time, 2.5, 1e-9);

    IVP_ASSERT_TRUE(ivp_time_event_queue_pop_due(&queue, 3.0, &out));
    IVP_ASSERT_NEAR(out.trigger_time, 3.0, 1e-9);

    ivp_time_event_queue_destroy(&queue);
}

IVP_TEST(time_event_queue_deduplicates_pair)
{
    ivp_time_event_queue_t queue;
    ivp_time_event_queue_init(&queue, NULL);

    ivp_time_event_t e;
    e.trigger_time = 5.0;
    e.obj_a = make_handle(10u, 1u);
    e.obj_b = make_handle(11u, 1u);
    e.type = IVP_TIME_EVENT_MINDIST_IMPACT;
    e.status_a = IVP_ST_BALL;
    e.status_b = IVP_ST_BALL;
    e.expected_gap = IVP_REAL(0.01);
    e.serial = 0u;

    IVP_ASSERT_TRUE(ivp_time_event_queue_push(&queue, &e));

    e.trigger_time = 2.0;
    IVP_ASSERT_TRUE(ivp_time_event_queue_push(&queue, &e));

    IVP_ASSERT_EQ(queue.count, 1);

    ivp_time_event_t out;
    IVP_ASSERT_TRUE(ivp_time_event_queue_pop_due(&queue, 2.1, &out));
    IVP_ASSERT_NEAR(out.trigger_time, 2.0, 1e-9);

    ivp_time_event_queue_destroy(&queue);
}

IVP_TEST(time_event_queue_remove_object)
{
    ivp_time_event_queue_t queue;
    ivp_time_event_queue_init(&queue, NULL);

    ivp_time_event_t a;
    ivp_time_event_t b;

    a.trigger_time = 1.0;
    a.obj_a = make_handle(20u, 1u);
    a.obj_b = make_handle(21u, 1u);
    a.type = IVP_TIME_EVENT_MINDIST_IMPACT;

    b = a;
    b.trigger_time = 1.2;
    b.obj_a = make_handle(22u, 1u);
    b.obj_b = make_handle(23u, 1u);

    IVP_ASSERT_TRUE(ivp_time_event_queue_push(&queue, &a));
    IVP_ASSERT_TRUE(ivp_time_event_queue_push(&queue, &b));
    IVP_ASSERT_EQ(queue.count, 2);

    ivp_time_event_queue_remove_object(&queue, make_handle(20u, 1u));
    IVP_ASSERT_EQ(queue.count, 1);

    ivp_time_event_t out;
    IVP_ASSERT_TRUE(ivp_time_event_queue_pop_due(&queue, 2.0, &out));
    IVP_ASSERT_EQ(out.obj_a.slot, 22u);

    ivp_time_event_queue_destroy(&queue);
}

int main(void)
{
    IVP_REGISTER(time_event_queue_orders_by_time);
    IVP_REGISTER(time_event_queue_deduplicates_pair);
    IVP_REGISTER(time_event_queue_remove_object);
    IVP_RUN_ALL();
}
