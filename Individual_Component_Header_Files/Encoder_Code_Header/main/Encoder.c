#include "Encoder.h"

unsigned long prev_update_time_test_motor1 = 0;
unsigned long pre_encoder_ticks_test_motor1 = 0;

unsigned long prev_update_time_test_motor2 = 0;
unsigned long pre_encoder_ticks_test_motor2 = 0;

// This creates the setting for the unit counter
// Essentially setting a limit to the range the counter can be
pcnt_unit_config_t unit_config = {
    .high_limit = PCNT_HIGH_LIMIT,
    .low_limit = PCNT_LOW_LIMIT,
};

// Here is the configuration for the channel
// We are only using the rising edge from the encoder
// to count the pulses for determing the RPM, which is
// why we set level to -1
pcnt_chan_config_t chan_a_config = {
    .edge_gpio_num = MOTOR_A_ENCODER,
    .level_gpio_num = -1,
};

// Repeat for motor B encoder
pcnt_chan_config_t chan_b_config = {
    .edge_gpio_num = MOTOR_B_ENCODER,
    .level_gpio_num = -1,
};

pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 1000,
};

void create_pcnt_for_motors(pcnt_unit_handle_t* motor_A, pcnt_unit_handle_t* motor_B){
    // Here we will create the counting unit, this will
    // be used to keep track of the rototary enocoder counts
    ESP_LOGI(TAG, "Create pcnt unit for Motor A");
   
    // Create PCNT unit with unit_config
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, motor_A));

    // Repeating the same step for Motor B
    ESP_LOGI(TAG, "Create pcnt unit for Motor B");
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, motor_B));

    // Next the glitch filter is going to be placed
    // this just makes sure we only read stable signals
    // similar to push button debouncing
    ESP_LOGI(TAG, "Setting Glitch Filters");
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*motor_A, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*motor_B, &filter_config));
}

void create_pcnt_channels_for_motors(pcnt_unit_handle_t* motor_A, pcnt_unit_handle_t* motor_B, pcnt_channel_handle_t* channel_A, pcnt_channel_handle_t* channel_B){
    // Next is setting up the actual gpio channel that will increment
    // or decrement the counter
    ESP_LOGI(TAG, "Creating pcnt channels");

    // Create the channel/gpio and tie it to unit counter
    ESP_ERROR_CHECK(pcnt_new_channel(*motor_A, &chan_a_config, channel_A));

    ESP_ERROR_CHECK(pcnt_new_channel(*motor_B, &chan_b_config, channel_B));

    // This section focuses on setting the logic/reaction 
    // to when the gpio/channel recieves a rising edge
    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");

    // On the rising edge (the second arg to pcnt_channel_set_edge_action)
    // the counter will increment while on the falling edge (third arg)
    // the counter will remain the same
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(*channel_A, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(*channel_B, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    // Even though we are not using level, the setting will be set to
    // keep the count going in the same direction, counting up to 1000,
    // just to be safe
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(*channel_A, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(*channel_B, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
}

void start_pcnt_for_motors(pcnt_unit_handle_t* motor_A, pcnt_unit_handle_t* motor_B){
    // Finally here is where the counters enabled, cleared to start at
    // zero, and started
    
    // Enable pcnt units for motor A and motor B
    ESP_LOGI(TAG, "enable pcnt units");
    ESP_ERROR_CHECK(pcnt_unit_enable(*motor_A));
    ESP_ERROR_CHECK(pcnt_unit_enable(*motor_B));

    // Clear the counter incase there is data left over
    ESP_LOGI(TAG, "clear pcnt units");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*motor_A));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*motor_B));

    // Start counters
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(*motor_A));
    ESP_ERROR_CHECK(pcnt_unit_start(*motor_B));
}


float getRPM_motor1(pcnt_unit_handle_t * motor_enc){
    int encoder_ticks = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(*motor_enc, (&encoder_ticks)));

    ESP_LOGI(TAG, "Encoder Count: %d", encoder_ticks);

    unsigned long current_time = esp_timer_get_time();
    unsigned long dt = current_time - prev_update_time_test_motor1;

    double dtm = (double)dt / 6000000;
    double delta_ticks = encoder_ticks - pre_encoder_ticks_test_motor1;

    prev_update_time_test_motor1 = current_time;
    pre_encoder_ticks_test_motor1 = encoder_ticks;

    //20 is the counts_per_rev_ for our encoder
    return ((delta_ticks / 20) / dtm);
}

float getRPM_motor2(pcnt_unit_handle_t * motor_enc){
    int encoder_ticks = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(*motor_enc, (&encoder_ticks)));

    ESP_LOGI(TAG, "Encoder Count: %d", encoder_ticks);

    unsigned long current_time = esp_timer_get_time();
    unsigned long dt = current_time - prev_update_time_test_motor2;

    double dtm = (double)dt / 6000000;
    double delta_ticks = encoder_ticks - pre_encoder_ticks_test_motor2;

    prev_update_time_test_motor2 = current_time;
    pre_encoder_ticks_test_motor2 = encoder_ticks;

    //20 is the counts_per_rev_ for our encoder
    return ((delta_ticks / 20) / dtm);
}