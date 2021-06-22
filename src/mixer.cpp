// Module to handle actuator input/output and mixing.

#include "props2.h"

#include "config.h"
#include "imu_mgr.h"
#include "pilot.h"

#include "mixer.h"

#include "setup_board.h"

// reset sas parameters to startup defaults
void mixer_t::sas_defaults() {
    config.stab_cfg.sas_rollaxis = true;
    config.stab_cfg.sas_pitchaxis = true;
    config.stab_cfg.sas_yawaxis = true;
    config.stab_cfg.sas_tune = false;

    config.stab_cfg.sas_rollgain = 0.2;
    config.stab_cfg.sas_pitchgain = 0.2;
    config.stab_cfg.sas_yawgain = 0.2;
    config.stab_cfg.sas_max_gain = 2.0;
};


// reset mixing parameters to startup defaults
// void mixer_t::mixing_defaults() {
//     config.actuators.mix_autocoord = false;
//     config.actuators.mix_throttle_trim = false;
//     config.actuators.mix_flap_trim = false;
//     config.actuators.mix_elevon = false;
//     config.actuators.mix_flaperon = false;
//     config.actuators.mix_vtail = false;
//     config.actuators.mix_diff_thrust = false;

//     config.actuators.mix_Gac = 0.5;       // aileron gain for autocoordination
//     config.actuators.mix_Get = -0.1;      // elevator trim w/ throttle gain
//     config.actuators.mix_Gef = 0.1;       // elevator trim w/ flap gain

//     config.actuators.mix_Gea = 1.0;       // aileron gain for elevons
//     config.actuators.mix_Gee = 1.0;       // elevator gain for elevons
//     config.actuators.mix_Gfa = 1.0;       // aileron gain for flaperons
//     config.actuators.mix_Gff = 1.0;       // flaps gain for flaperons
//     config.actuators.mix_Gve = 1.0;       // elevator gain for vtail
//     config.actuators.mix_Gvr = 1.0;       // rudder gain for vtail
//     config.actuators.mix_Gtt = 1.0;       // throttle gain for diff thrust
//     config.actuators.mix_Gtr = 0.1;       // rudder gain for diff thrust
// };


void mixer_t::update_matrix(message::config_mixer_t *mix_config ) {
    M.setIdentity();            // straight pass through default

    // note: M(output_channel, input_channel)
    // note: elevon and flaperon mixing are mutually exclusive

    PropertyNode autocoord_node = PropertyNode("/config/mixer/auto_coordination");
    PropertyNode throttletrim_node = PropertyNode("/config/mixer/throttle_trim");
    PropertyNode flaptrim_node = PropertyNode("/config/mixer/flap_trim");
    PropertyNode elevon_node = PropertyNode("/config/mixer/elevon");
    PropertyNode flaperon_node = PropertyNode("/config/mixer/flaperon");
    PropertyNode vtail_node = PropertyNode("/config/mixer/vtail");
    PropertyNode diffthrust_node = PropertyNode("/config/mixer/diff_thrust");
    hal.scheduler->delay(100);

    if ( autocoord_node.getBool("enable") ) {
        M(3,1) = -autocoord_node.getFloat("gain1");
        if (vtail_node.getBool("enable") && !elevon_node.getBool("enable")) {
            M(2,1) = autocoord_node.getFloat("gain1");
        }
    }
    if ( throttletrim_node.getBool("enable") ) {
        M(2,0) = throttletrim_node.getFloat("gain1");
    }
    if ( flaptrim_node.getBool("enable") ) {
        M(2,4) = flaptrim_node.getFloat("gain1");
        if ( vtail_node.getBool("enable") && !elevon_node.getBool("enable")) {
            M(3,4) = flaptrim_node.getFloat("gain1");
        }
    }
    if ( elevon_node.getBool("enable") ) {
        M(1,1) = elevon_node.getFloat("gain1");
        M(1,2) = elevon_node.getFloat("gain2");
        M(2,1) = elevon_node.getFloat("gain1");
        M(2,2) = -elevon_node.getFloat("gain2");
    } else if ( flaperon_node.getBool("enable") ) {
        M(1,1) = flaperon_node.getFloat("gain1");
        M(1,4) = flaperon_node.getFloat("gain2");
        M(4,1) = -flaperon_node.getFloat("gain1");
        M(4,4) = flaperon_node.getFloat("gain2");
    }
    // vtail mixing can't work with elevon mixing
    if ( vtail_node.getBool("enable") && !elevon_node.getBool("enable") ) {
        M(2,2) = vtail_node.getFloat("gain1");
        M(2,3) = vtail_node.getFloat("gain2");
        M(3,2) = vtail_node.getFloat("gain1");
        M(3,3) = -vtail_node.getFloat("gain2");
    }
    if ( diffthrust_node.getBool("eanbled") ) {
        // fixme: never tested in the wild (need to think through channel assignments)
        // outputs[0] = mix_config.mix_Gtt * throttle_cmd + mix_config.mix_Gtr * rudder_cmd;
        // outputs[5] = mix_config.mix_Gtt * throttle_cmd - mix_config.mix_Gtr * rudder_cmd;
    }
}

void mixer_t::print_mixer_matrix() {
    console->printf("Mixer Matrix:\n");
    for ( int i = 0; i < MAX_RCOUT_CHANNELS; i++ ) {
        console->printf("  ");
        for ( int j = 0; j < MAX_RCOUT_CHANNELS; j++ ) {
            if ( M(i,j) >= 0 ) {
                console->printf(" ");
            }
            console->printf("%.2f ", M(i,j));
        }
        console->printf("\n");
    }
}
void mixer_t::setup() {
    outputs.setZero();
    M = Eigen::Matrix<float, MAX_RCOUT_CHANNELS, MAX_RCOUT_CHANNELS, Eigen::RowMajor>(config.mixer_matrix_cfg.matrix);
    M.setIdentity();
    message::config_mixer_t config_mixer;
    update_matrix(&config_mixer);
    print_mixer_matrix();
    hal.scheduler->delay(100);
}

// compute the stability damping in normalized command/input space
// which simplifies mixing later
void mixer_t::sas_update() {
    float tune = 1.0;
    if ( config.stab_cfg.sas_tune ) {
        tune = config.stab_cfg.sas_max_gain * pilot.manual_inputs[7];
        if ( tune < 0.0 ) {
            tune = 0.0;
        } else if ( tune > 2.0 ) {
            tune = 2.0;
        }
    }

    if ( config.stab_cfg.sas_rollaxis ) {
        inputs[1] -= tune * config.stab_cfg.sas_rollgain * imu_mgr.get_p_cal();
    }
    if ( config.stab_cfg.sas_pitchaxis ) {
        inputs[2] += tune * config.stab_cfg.sas_pitchgain * imu_mgr.get_q_cal();
    }
    if ( config.stab_cfg.sas_yawaxis ) {
        inputs[3] += tune * config.stab_cfg.sas_yawgain * imu_mgr.get_r_cal();
    }
}

// compute the actuator (servo) values for each channel.  Handle all
// the requested mixing modes here.
void mixer_t::mixing_update() {
    outputs = M * inputs;
    
    if ( pilot.throttle_safety() ) {
        outputs[0] = 0.0;
    }
}

void mixer_t::update() {
    // the pilot.get_* interface is smart to return manual
    // vs. autopilot depending on switch state.
    inputs << pilot.get_throttle(), pilot.get_aileron(), pilot.get_elevator(),
        pilot.get_rudder(), pilot.get_flap(), pilot.get_gear(),
        pilot.get_ch7(), pilot.get_ch8();
    
    sas_update();
    mixing_update();
}
