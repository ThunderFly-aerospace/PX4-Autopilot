#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_value.h>

__EXPORT int platform_open_main(int argc, char *argv[]);

int platform_open_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");

    
    /* advertise attitude topic */
    struct debug_value_s takeoff_information;
    memset(&takeoff_information, 0, sizeof(takeoff_information));
    orb_advert_t att_pub = orb_advertise(ORB_ID(debug_value), &takeoff_information);

	takeoff_information.timestamp = hrt_absolute_time();
	takeoff_information.value = 3;

    orb_publish(ORB_ID(debug_value), att_pub, &takeoff_information);
   
    PX4_INFO("published exiting");

    return 0;
}
