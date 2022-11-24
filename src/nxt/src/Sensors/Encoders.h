#ifndef SRC_GLOBALS_H_
#define SRC_GLOBALS_H_

#include "stdint.h"
#include "fix16.h"


#define MAX_ENC_GROUP_SIZE 4
#define ENC_TYPE_RELATIVE 0 
#define ENC_TYPE_ABSOLUTE 1 

struct encoder_group
{
    uint8_t num_encs;                           // Number of physical encoders in this group
    uint8_t enc_type[MAX_ENC_GROUP_SIZE];       // Relative or absolute encoder
    uint32_t enc_port[MAX_ENC_GROUP_SIZE];      // Physical port to which the encoder is attached
    fix16_t r_ei_to_out[MAX_ENC_GROUP_SIZE];    // Mechanical ratio between each encoder and the virtual output shaft
    fix16_t r_out_to_ei[MAX_ENC_GROUP_SIZE];    // Inverse of r_ei_to_out
    *get_angle(uint32_t)[MAX_ENC_GROUP_SIZE];   // Function which samples an encoder and returns the result in radians (fix16_t)
    *set_angle(uint32_t, fix16_t)[MAX_ENC_GROUP_SIZE]; // Function which sets an encoder to a specific angle at its current position
    *correlate(uint8_t, *fix16_t, *uint8_t);    // Function which returns a single angle (fix16_t) at the virtual output, best representing the group state
}


// PUBLIC FUNCTIONS:
fix16_t enc_get_angle(struct encoder_group * enc_group);
void enc_set_angle(struct encoder_group * enc_group, fix16_t out_angle);


// CORRELATION FUNCTIONS:
fix16_t enc_correlate_use_only_first(uint8_t num_encs, fix16_t * out_angles, uint8_t * enc_types);
fix16_t enc_correlate_use_only_first_absolute(uint8_t num_encs, fix16_t * out_angles, uint8_t * enc_types);
fix16_t enc_correlate_average_all(uint8_t num_encs, fix16_t * out_angles, uint8_t * enc_types);

// HARDWARE-SPECIFIC INTERFACES:
fix16_t enc_get_angle_builtin(uint32_t port);
fix16_t enc_get_angle_hitechnic(uint32_t port);
void enc_set_angle_builtin(uint32_t port, fix16_t ei_angle);
void enc_set_angle_hitechnic(uint32_t port, fix16_t ei_angle);


#endif // SRC_GLOBALS_H