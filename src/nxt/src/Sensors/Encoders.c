#include "Encoders.h"


// PUBLIC FUNCTIONS


// RETURNS: out_angle, angle (radians) at the output shaft
fix16_t enc_get_angle(struct encoder_group * enc_group){
    uint8_t num_encs = enc_group->num_encs;
    fix16_t out_angles [num_encs];

    for(uint32_t i=0; i<num_encs; i++){     // sample every encoder in the group
        uint32_t port = enc_group->enc_ports[i];
        fix16_t r_ei_to_out = enc_group->r_ei_to_out[i];
        fix16_t ei_angle = (enc_group->get_angle[i])(port);
        out_angles[i] = fix16_mul(ei_angle, r_ei_to_out):
    }

    fix16_t out_angle = enc_group->correlate(
        num_encs,
        &out_angles
        enc_group->enc_types,
    );
    return out_angle;
}

void enc_set_angle(struct encoder_group * enc_group, fix16_t out_angle){
    uint8_t num_encs = enc_group->num_encs;

    for(uint32_t i=0; i<num_encs; i++){     // calculate angle for every encoder in the group
        uint32_t port = enc_group->enc_ports[i];
        fix16_t r_out_to_ei = enc_group->r_out_to_ei[i];
        fix16_t ei_angle = fix16_mul(out_angle, r_out_to_ei);
        (enc_group->set_angle[i])( port, ei_angle );
    }
}


// CORRELATION FUNCTIONS
static fix16_t enc_correlate_use_only_first(uint8_t num_encs, fix16_t * out_angles, uint8_t * enc_types){
    fix16_t out_angle = out_angles[0];
    return out_angle;
}

fix16_t enc_correlate_use_only_first_absolute(uint8_t num_encs, fix16_t * out_angles, uint8_t * enc_types){
    for(uint8_t i=0; i<num_encs; i++){
        if(enc_types[i] == ENC_TYPE_ABSOLUTE){
            return out_angles[i]
}

fix16_t enc_correlate_average_all(uint8_t num_encs, fix16_t * out_angles, uint8_t * enc_types){
    fix16_t out_angle = F16(0.0);
    for(uint8_t i=0; i<num_encs; i++){
        out_angle = fix16_add(out_angle, out_angles[i]);
    }
    out_angle = fix16_div(out_angle, fix16_from_int(num_encs))
    return out_angle;
}


// HELPER FUNCTIONS FOR SPECIFIC ENCODER TYPES
fix16_t enc_get_angle_builtin(uint32_t port){
	int32_t cnt = nxt_motor_get_count(port);
    fix16_t ei_angle = fix16_deg_to_rad(fix16_from_int(cnt));
    return ei_angle;
}


fix16_t enc_get_angle_hitechnic(uint32_t port){
    //TODO: Interface with Hitechnic angle sensor driver
	int32_t cnt = 0;
    fix16_t ei_angle = fix16_deg_to_rad(fix16_from_int(cnt));
    return ei_angle;
}

void enc_set_angle_builtin(uint32_t port, fix16_t ei_angle){
    int32_t cnt = fix16_to_int(fix16_rad_to_deg(ei_angle));
    nxt_motor_set_count(port, cnt);
}

void enc_set_angle_hitechnic(uint32_t port, fix16_t ei_angle){
    int32_t cnt = fix16_to_int(fix16_rad_to_deg(ei_angle));
    //TODO: Interface with Hitechnic angle sensor driver
}
