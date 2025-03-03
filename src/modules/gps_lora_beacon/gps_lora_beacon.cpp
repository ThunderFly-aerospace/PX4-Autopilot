#include <math.h>
#include <poll.h>
#include <px4_log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/outgoing_lora_message.h>
#include <drivers/drv_hrt.h>

extern "C" __EXPORT int gps_lora_beacon_main(int argc, char *argv[]);

uint32_t pack_latlon(double v)
{
    return (int32_t) (v*4194304.0);
}

enum {
    LATLON_OK = (1<<0),
    ALT_OK    = (1<<1),
    COURSE_OK = (1<<2),
    SPEED_OK  = (1<<3)
};

void dbg_print_gps_struct(struct sensor_gps_s raw)  
{
    PX4_INFO("Lat/Lon (deg): \t%4.6f\t%4.6f",
             raw.latitude_deg,
             raw.longitude_deg);
    PX4_INFO("Alt (m): \t%4.1f",  raw.altitude_msl_m);
    PX4_INFO("Fix: \t%d", raw.fix_type);
    PX4_INFO("Eph/Epv (m): \t%2.2f\t%2.2f", (double) raw.eph, (double) raw.epv);
    PX4_INFO("Speed Ground/Down (m/s): \t%2.2f\t%2.2f (valid=%d)",
             (double) raw.vel_m_s, (double) raw.vel_d_m_s, raw.vel_ned_valid);
    PX4_INFO("Course (deg): \t%3.1f", (double) raw.cog_rad*180/M_PI);
    PX4_INFO("Sats Used: \t%d", raw.satellites_used);
}

int fill_lora_msg(struct sensor_gps_s *in, uint8_t *out, int nmaxbytes) {
    uint8_t flags;
    uint8_t *p = out + 1;

    if (nmaxbytes < 16)
        return -1;
    flags = 0;
    uint32_t lat = pack_latlon(in->latitude_deg);
    *p++ = lat >> 24;
    *p++ = lat >> 16;
    *p++ = lat >> 8;
    *p++ = lat;
    uint32_t lon = pack_latlon(in->longitude_deg);
    *p++ = lon >> 24;
    *p++ = lon >> 16;
    *p++ = lon >> 8;
    *p++ = lon;
    if (in->fix_type >= 2)
        flags |= LATLON_OK;
    int32_t age_s = (hrt_absolute_time() - in->timestamp) / 1000000;
    if (age_s < 0)      age_s = 0;
    if (age_s > 0xffff) age_s = 0xffff;
    *p++ = age_s >> 8;
    *p++ = age_s;
    int32_t alt_m = in->altitude_msl_m;
    if (alt_m < -0x7ffe) alt_m = -0x7ffe;
    if (alt_m >  0x7ffe) alt_m =  0x7ffe;
    *p++ = alt_m >> 8;
    *p++ = alt_m;
    if (in->fix_type >= 3)
        flags |= ALT_OK;
    int16_t course = in->cog_rad*180*64/M_PI_F;
    *p++ = course >> 8;
    *p++ = course;
    if (!isnan(in->cog_rad))
        flags |= COURSE_OK;
    int16_t speed = in->vel_m_s*16;
    *p++ = speed >> 8;
    *p++ = speed;
    if (!isnan(in->vel_m_s))
        flags |= COURSE_OK;
    *out = flags;

    return p - out;
}

int period_s;

int gps_lora_beacon_thread_main(int argc, char *argv[])
{
    uint64_t last_packet = 0;
    struct sensor_gps_s last_good_fix;
    struct outgoing_lora_message_s outgoing;
    memset(&last_good_fix, 0, sizeof(last_good_fix));

    PX4_INFO("Hello from GPS Lora Beacon!");

    int gps_sub_fd = orb_subscribe(ORB_ID(sensor_gps));
    orb_set_interval(gps_sub_fd, 200); /* limit the update rate to 5 Hz */

    orb_advert_t outgoing_pub = orb_advertise(ORB_ID(outgoing_lora_message), &outgoing);

    px4_pollfd_struct_t fds[] = {
        { .fd = gps_sub_fd,   .events = POLLIN },
    };

    int error_counter = 0;

    while (true) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;

        } else {
            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct sensor_gps_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(sensor_gps), gps_sub_fd, &raw);
                if (raw.fix_type >= 2)
                    memcpy(&last_good_fix, &raw, sizeof(raw));

            }
        }

        if ((hrt_absolute_time() - last_packet) > ((uint64_t) period_s)*1000*1000) {
            outgoing.len = fill_lora_msg(&last_good_fix, outgoing.data, sizeof(outgoing.data));
            orb_publish(ORB_ID(outgoing_lora_message), outgoing_pub, &outgoing);
            last_packet = hrt_absolute_time();
        }
    }

    PX4_INFO("exiting");

    return OK;
}

static void usage() {
    fprintf(stderr, "usage: gps_lora_beacon -p PERIOD_S\n");
}

extern "C"
int
gps_lora_beacon_main(int argc, char *argv[])
{
    period_s = 30;
    int opt;
    int moptind = 1;
    const char* moptarg;
    while ((opt = px4_getopt(argc, argv, "p:", &moptind, &moptarg)) != EOF) {
        switch (opt) {
        case 'p':
            period_s = atoi(moptarg);
            if (period_s < 0) {
                usage();
                return 1;
            }
            break;
        default:
            usage();
            return 1;
        }
    }

    px4_task_spawn_cmd("beacon",
        SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5,
        2000, gps_lora_beacon_thread_main, (char *const *)nullptr);

    return 0;
}
