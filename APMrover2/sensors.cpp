// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

void Rover::init_barometer(bool full_calibration)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Calibrating barometer");
    if (full_calibration) {
        barometer.calibrate();
    } else {
        barometer.update_calibration();
    }
    gcs_send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");
}

void Rover::init_sonar(void)
{
    sonar.init();
}

// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
void Rover::read_battery(void)
{
    battery.read();
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Rover::read_receiver_rssi(void)
{
    receiver_rssi = rssi.read_receiver_rssi_uint8();
}

//Calibrate compass
void Rover::compass_cal_update() {
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }
}

// Accel calibration

void Rover::accel_cal_update() {
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them    float trim_roll, trim_pitch;
    float trim_roll,trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
}

// read the sonars
void Rover::read_sonars(void)
{
    sonar.update();

    if (sonar.status() == RangeFinder::RangeFinder_NotConnected) {
        // this makes it possible to disable sonar at runtime
        return;
    }

    if (sonar.has_data(1)) {
        // we have two sonars
        obstacle.sonar1_distance_cm = sonar.distance_cm(0);
        obstacle.sonar2_distance_cm = sonar.distance_cm(1);
        if (obstacle.sonar1_distance_cm < (uint16_t)g.sonar_trigger_cm &&
            obstacle.sonar1_distance_cm < (uint16_t)obstacle.sonar2_distance_cm)  {
            // we have an object on the left
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Sonar1 obstacle %u cm",
                                  (unsigned)obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = AP_HAL::millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        } else if (obstacle.sonar2_distance_cm < (uint16_t)g.sonar_trigger_cm) {
            // we have an object on the right
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Sonar2 obstacle %u cm",
                                  (unsigned)obstacle.sonar2_distance_cm);
            }
            obstacle.detected_time_ms = AP_HAL::millis();
            obstacle.turn_angle = -g.sonar_turn_angle;
        }
    } else {
        // we have a single sonar
        obstacle.sonar1_distance_cm = sonar.distance_cm(0);
        obstacle.sonar2_distance_cm = 0;
        if (obstacle.sonar1_distance_cm < (uint16_t)g.sonar_trigger_cm)  {
            // obstacle detected in front 
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Sonar obstacle %u cm",
                                  (unsigned)obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = AP_HAL::millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        }
    }

    Log_Write_Sonar();

    // no object detected - reset after the turn time
    if (obstacle.detected_count >= g.sonar_debounce &&
        AP_HAL::millis() > obstacle.detected_time_ms + g.sonar_turn_time*1000) { 
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Obstacle passed");
        obstacle.detected_count = 0;
        obstacle.turn_angle = 0;
    }
}

/*
  update AP_Button
 */
void Rover::button_update(void)
{
    button.update();
}

void copy(uint8_t * origin, int offset, int length, uint8_t * destination)
{
    int i;
    for(i = 0; i < length; i ++)
	    destination[i] = origin[i + offset];
}

void Rover::read_external_data(void)
{

/*
    uint8_t * local_buffer;
    local_buffer = new uint8_t[19];
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> arduino = hal.i2c_mgr->get_device(1, 7);
    if( arduino->transfer(nullptr, 0, local_buffer, i2c_buffer_length) )
    {
	for(uint32_t i = 0; i < i2c_buffer_length; i ++)
	    i2c_buffer[i] = local_buffer[i];
	new_data_received = true;
    }

    delete [] local_buffer;
*/
	int local_buffer_length = 24;
	uint8_t local_buffer[local_buffer_length];

	union float2bytes { float f; uint8_t b[sizeof(float)]; };
	float2bytes f2b;

	union int2bytes { uint32_t i; uint8_t b[sizeof(uint32_t)]; };
	int2bytes i2b;

  	float values[5];
   	uint32_t battery_status = 9999;

   	AP_HAL::OwnPtr<AP_HAL::I2CDevice> arduino = hal.i2c_mgr->get_device(1, 7);

        if( arduino->transfer(nullptr, 0, local_buffer, local_buffer_length) )
	{
		new_data_received = true;
		
		copy(local_buffer, 0, 4, f2b.b);
		i2c_buffer.voltage = f2b.f;

		copy(local_buffer, 4, 4, f2b.b);
		i2c_buffer.current = f2b.f;
		
		copy(local_buffer, 8, 4, f2b.b);
		i2c_buffer.air_temperature = f2b.f;
	
		copy(local_buffer, 12, 4, f2b.b);
		i2c_buffer.water_temperature = f2b.f;	
	
		copy(local_buffer, 16, 4, f2b.b);
		i2c_buffer.humidity = f2b.f;
	
		copy(local_buffer, 20, 4, i2b.b);
		i2c_buffer.battery_status = i2b.i;
	}
}
