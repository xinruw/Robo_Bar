/*
 * Description:  A really simple controller which moves the MyBot robot,
 *               avoids the walls and turns the camera on.
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>

#define SPEED 60
#define TIME_STEP 64

int main()
{
  wb_robot_init(); /* necessary to initialize webots stuff */

  /* Get and enable the distance sensors. */
  WbDeviceTag ds0 = wb_robot_get_device("ds0");
  WbDeviceTag ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

    /* get and enable camera */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 2 * TIME_STEP);

  while(wb_robot_step(TIME_STEP)!=-1) {

    /* Get distance sensor values */
    double ds0_value = wb_distance_sensor_get_value(ds0);
    double ds1_value = wb_distance_sensor_get_value(ds1);

    /* This is used to refresh the camera. */
    wb_camera_get_image(camera);

    /* Compute the motor speeds */
    double left_speed, right_speed;
    if (ds1_value > 500) {

        /*
         * If both distance sensors are detecting something, this means that
         * we are facing a wall. In this case we need to move backwards.
         */
        if (ds0_value > 500) {
            left_speed = -SPEED;
            right_speed = -SPEED / 2;
        } else {

            /*
             * We turn proportionnaly to the sensors value because the
             * closer we are from the wall, the more we need to turn.
             */
            left_speed = -ds1_value / 10;
            right_speed = (ds0_value / 10) + 5;
        }
    } else if (ds0_value > 500) {
        left_speed = (ds1_value / 10) + 5;
        right_speed = -ds0_value / 10;
    } else {

        /*
         * If nothing has been detected we can move forward at maximal speed.
         */
        left_speed = SPEED;
        right_speed = SPEED;
    }

    /* Set the motor speeds. */
    wb_differential_wheels_set_speed(left_speed, right_speed);
  }

  return 0;
}
