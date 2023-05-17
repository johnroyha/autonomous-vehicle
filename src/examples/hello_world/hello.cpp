#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <uORB/Publication.hpp>

#include <uORB/topics/led_control.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/input_rc.h>


#define DC_MOTOR (0u)
#define SERVO_MOTOR (1u)

extern "C" __EXPORT int hello_world_main(int argc, char *argv[]);
int hello_world_main(int argc, char *argv[])
{
    px4_sleep(2);

    // Configure motors
    test_motor_s dc_motor;
    test_motor_s servo_motor;

    // Subscribe to debug_value topic (to get data from RPI)
    debug_value_s debug_data;
    int debug_handle = orb_subscribe(ORB_ID(debug_value));
    orb_set_interval(debug_handle, 500);
    led_control_s led_control;

    // Publish to test_motor and led_control topic (to control motors and change leds)
    uORB::Publication<led_control_s> led_control_pub(ORB_ID(led_control));
    uORB::Publication<test_motor_s> test_motor_pub(ORB_ID(test_motor));

    // Set initial values of motors
    set_motor(&dc_motor, 0.5);
    set_motor(&servo_motor, 0.5);

    // Set initialization of RC
    input_rc_s rc_data;
    double dc_throttle = 0;
    double servo_throttle = 0;
    double teleop_auton_mode = 0;
    int input_rc_handle = orb_subscribe(ORB_ID(input_rc));
    orb_set_interval(input_rc_handle, 200);

    int message;
    while (1)
    {
        orb_copy(ORB_ID(input_rc), input_rc_handle, &rc_data);
        teleop_auton_mode = rc_data.values[5];

        if (teleop_auton_mode < 0.5) {
            PX4_INFO("Teleop mode");
            dc_throttle = rc_data.values[2];
            servo_throttle = rc_data.values[0];
            dc_throttle = (dc_throttle - 1024.0) / 979.0;
            servo_throttle = ((servo_throttle - 1024.0) / 979.0)*(-1) + 1;

            set_motor(&dc_motor, dc_throttle);
            set_motor(&servo_motor, servo_throttle);
            continue;
        }
        else {
            PX4_INFO("Auton mode");
            orb_copy(ORB_ID(debug_value), debug_handle, &debug_data);
            led_control.timestamp = hrt_absolute_time();
            led_control.color = led_control_s::COLOR_GREEN;
            led_control.priority = led_control_s::MAX_PRIORITY;
            led_control.led_mask = 0xff;

            double motor_speed = 0.0;
            double motor_angle = 0.0;

            message = debug_data.ind;
            switch(message) {
                case(0): // Full speed forward
                    motor_speed = 1.0;
                    motor_angle = 0.5;
                    led_control.color = led_control_s::COLOR_GREEN;
                    led_control.mode = led_control_s::MODE_ON;
                    break;
                case(1): // Stop
                    motor_speed = 0.5;
                    motor_angle = 0.5;
                    led_control.color = led_control_s::COLOR_BLUE;
                    led_control.mode = led_control_s::MODE_ON;
                    break;
                case(2): // Slow Left
                    motor_speed = 0.60;
                    motor_angle = 0.0;
                    led_control.color = led_control_s::COLOR_YELLOW;
                    led_control.mode = led_control_s::MODE_ON;
                    break;
                case(3): // Slow Right
                    motor_speed = 0.60;
                    motor_angle = 1.0;
                    led_control.color = led_control_s::COLOR_PURPLE;
                    led_control.mode = led_control_s::MODE_ON;
                    break;
                case(4): // Slow Forward
                    motor_speed = 0.60;
                    motor_angle = 0.5;
                    led_control.color = led_control_s::COLOR_RED;
                    led_control.mode = led_control_s::MODE_ON;
                    break;
                default: // None of the above, should not happen
                    motor_speed = 0.0;
                    motor_angle = 0.5;
                    led_control.color = led_control_s::COLOR_WHITE;
                    led_control.mode = led_control_s::MODE_ON;
                    break;
            }
        led_control_pub.publish(led_control);
        set_motor(&dc_motor, motor_speed);
        set_motor(&servo_motor, motor_angle);
        }
    }
    PX4_INFO("The motor will be stopped");
    set_motor(&dc_motor, 0.5);
    set_motor(&servo_motor, 0.5);

    return 0;
}

//Set the motor speed to a specified value
void set_motor(test_motor_s *motor, float value) {
    motor.timestamp = hrt_absolute_time();
    motor.motor_number = DC_MOTOR;
    motor.value = (float)value;
    motor.action = test_motor_s::ACTION_RUN;
    motor.driver_instance = 0;
    motor.timeout_ms = 0;
    test_motor_pub.publish(motor);
    return;
}
