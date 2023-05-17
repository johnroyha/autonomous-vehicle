#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <uORB/topics/input_rc.h>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/test_motor.h>

#define DC_MOTOR (0u)
#define SERVO_MOTOR (1u)

extern "C" __EXPORT int hello_world_main(int argc, char *argv[]);
int hello_world_main(int argc, char *argv[]) {
    test_motor_s dc_motor;
    test_motor_s servo_motor;
    // double dc_motor_value = 0;
    // double servo_motor_value = 0;
    int input_rc_handle;
    input_rc_s rc_data;
    double dc_throttle = 0;
    double servo_throttle = 0;

    uORB::Publication<test_motor_s> test_motor_pub(ORB_ID(test_motor));

    input_rc_handle = orb_subscribe(ORB_ID(input_rc));
    orb_set_interval(input_rc_handle, 200);
    while(1) {
        orb_copy(ORB_ID(input_rc), input_rc_handle, &rc_data);
        servo_throttle = rc_data.values[0];
        dc_throttle = rc_data.values[2];

        servo_throttle = ((servo_throttle - 1024.0) / 979.0)*(-1) + 1;
        dc_throttle = (dc_throttle - 1024.0) / 979.0;

        // PX4_INFO("DC MOTOR: Enter speed value (0 to 1). If you enter a value outside the range, the motor will be stopped and the application will be terminated.");
        // scanf("%lf", &dc_motor_value);
        // if(dc_motor_value > 1.0 || dc_motor_value < 0) {
        //     break;
        // }

        PX4_INFO("DC Motor speed is %f", dc_throttle);
        dc_motor.timestamp = hrt_absolute_time();
        dc_motor.motor_number = DC_MOTOR;
        dc_motor.value = (float)dc_throttle;
        dc_motor.action = test_motor_s::ACTION_RUN;
        dc_motor.driver_instance = 0;
        dc_motor.timeout_ms = 0;

        test_motor_pub.publish(dc_motor);

        // PX4_INFO("SERVO: Enter speed value (0 to 1). If you enter a value outside the range, the motor will be stopped and the application will be terminated.");
        // scanf("%lf", &servo_motor_value);
        // if(servo_motor_value > 1.0 || servo_motor_value < 0) {
        //     break;
        // }

        PX4_INFO("Servo Motor speed is %f", servo_throttle);
        servo_motor.timestamp = hrt_absolute_time();
        servo_motor.motor_number = SERVO_MOTOR;
        servo_motor.value = (float)servo_throttle;
        servo_motor.action = test_motor_s::ACTION_RUN;
        servo_motor.driver_instance = 0;
        servo_motor.timeout_ms = 0;

        test_motor_pub.publish(servo_motor);

        // PX4_INFO("CH1 = %f, CH2 = %f, CH3 = %f, CH4 = %f, CH5 = %f",
		// 		(double )rc_data.values[0],
		// 		(double )rc_data.values[1],
		// 		(double )rc_data.values[2],
		// 		(double )rc_data.values[3],
		// 		(double )rc_data.values[4]);
        // PX4_INFO("CH6 = %f, CH7 = %f, CH8 = %f, CH9 = %f, CH10 = %f\n",
        //         (double )rc_data.values[5],
		// 		(double )rc_data.values[6],
		// 		(double )rc_data.values[7],
		// 		(double )rc_data.values[8],
		// 		(double )rc_data.values[9]);

        px4_usleep(200000);
    }

    PX4_INFO("The motor will be stopped");
    dc_motor.timestamp = hrt_absolute_time();
    dc_motor.motor_number = SERVO_MOTOR;
    dc_motor.value = 0.5;
    dc_motor.driver_instance = 0;
    dc_motor.timeout_ms = 0;

    servo_motor.timestamp = hrt_absolute_time();
    servo_motor.motor_number = SERVO_MOTOR;
    servo_motor.value = 0.5;
    servo_motor.driver_instance = 0;
    servo_motor.timeout_ms = 0;

    test_motor_pub.publish(dc_motor);
    test_motor_pub.publish(servo_motor);


    return 0;
}
