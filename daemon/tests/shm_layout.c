/* shm_layout - print the C layout of so101_shm_t as JSON, so the Python client
 * test (clients/python/test_layout.py) can check it decodes the same bytes. */
#include "so101_shm.h"

#include <stdio.h>

#define F(type, field) printf("  \"%s\": %zu,\n", #field, offsetof(type, field))

int main(void)
{
    printf("{\n");
    printf("  \"version\": %d,\n  \"shm_size\": %zu,\n  \"sample_size\": %zu,\n",
           SO101_SHM_VERSION, sizeof(so101_shm_t), sizeof(so101_sample_t));
    F(so101_shm_t, magic); F(so101_shm_t, joint_names); F(so101_shm_t, seq);
    F(so101_shm_t, s); F(so101_shm_t, mode_req); F(so101_shm_t, faults);
    F(so101_shm_t, sup_beat); F(so101_shm_t, cmd); F(so101_shm_t, ack);
    F(so101_sample_t, mode); F(so101_sample_t, leader_pos); F(so101_sample_t, follower_volt);
    F(so101_sample_t, leader_norm); F(so101_sample_t, follower_norm);
    F(so101_sample_t, overruns); F(so101_sample_t, leader_streak);
    printf("  \"ramping\": %zu\n}\n", offsetof(so101_sample_t, ramping));
    return 0;
}
