#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>

#include <px4_arch/micro_hal.h>

#include <stm32_gpio.h>
#include <stm32_exti.h>

#include "tflora.hpp"

#undef ASSERT
extern "C"{
#include "lmic/lmic.h"
}


extern TFLORA* tflora;

// Datasheet defins typical times until busy goes low. Most are < 200us,
// except when waking up from sleep, which typically takes 3500us. Since
// we cannot know here if we are in sleep, we'll have to assume we are.
// Since 3500 is typical, not maximum, wait a bit more than that.
static unsigned long MAX_BUSY_TIME = 5000;

// -----------------------------------------------------------------------------
// I/O

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getJoinEui (u1_t* /* buf */) { }
void os_getDevEui (u1_t* /* buf */ ) { }
void os_getNwkKey (u1_t* /* buf */) { }

// The region to use, this just uses the first one (can be changed if
// multiple regions are enabled).
u1_t os_getRegion (void) { return LMIC_regionCode(0); }

static void hal_io_init () {
}

// rx = 0, tx = 1, off = -1
void hal_ant_switch (u1_t val) {
}

// set radio RST pin to given value (or keep floating!)
bool hal_pin_rst (u1_t val) {
    return false;
}

void hal_irqmask_set (int /* mask */) {
    // Not implemented
}

void hal_pin_busy_wait (void) {
  if(tflora==nullptr)
  {
    printf("tflora empty\n");
    return;
  }

  tflora->wait_busy_pin(); 
}

#if defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)
bool hal_dio3_controls_tcxo (void) {
    return false; // lmic_pins.tcxo == LMIC_CONTROLLED_BY_DIO3;
}
bool hal_dio2_controls_rxtx (void) {
    return false; // lmic_pins.tx == LMIC_CONTROLLED_BY_DIO2;
}
#endif // defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)

// -----------------------------------------------------------------------------
// SPI

void hal_spi_init () {
}

void hal_spi_select (int on) {
    if (on != 0) 
      tflora->spi_select();
    else 
      tflora->spi_deselect();    
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    if(tflora==nullptr)
    {
      printf("tflora empty\n");
      return 0;
    }

    return tflora->spi_transfer(out);
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
    // Nothing to do
}

u4_t hal_ticks () {
    return hrt_absolute_time() / US_PER_OSTICK;
}

u8_t hal_xticks (void) {
    // TODO
    return hal_ticks();
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    if (delta > 0)
    {
        px4_usleep(delta*US_PER_OSTICK); //   delta/ time
    }
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

void hal_disableIRQs () {

}

void hal_enableIRQs () {

}

//unused
u1_t hal_sleep (u1_t type, u4_t target_time) {
    printf("hal sleep... fuj!");

/*    if (type == HAL_SLEEP_FOREVER) {
        px4_sem_wait(&sem_wakeup);
        return 0;
    }

    s4_t delta = delta_time(target_time);
    if (delta < 10)
        return 0;

    timespec abstime;
    if (clock_gettime(CLOCK_REALTIME, &abstime) != 0) {
        hal_debug_str("clock_gettime() in hal_speed failed\n");
        hal_failed();
    }
    const unsigned billion = 1000 * 1000 * 1000;
    uint64_t nsecs = abstime.tv_nsec + ((uint64_t) delta)*1000*US_PER_OSTICK;
    abstime.tv_sec += nsecs / billion;
    nsecs -= (nsecs / billion) * billion;
    abstime.tv_nsec = nsecs;

    int ret;
    while ((ret = sem_timedwait(&sem_wakeup, &abstime)) == -1 && errno == EINTR);
*/
    return 1;
}

void hal_watchcount (int /* cnt */) {
    // Not implemented
}

// -----------------------------------------------------------------------------
// DEBUG

#ifdef CFG_DEBUG
static void hal_debug_init() {
}


void hal_debug_str (const char* str) {
    printf("%s", str);
}

void hal_debug_led (int val) {
}
#endif // CFG_DEBUG

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
void hal_printf_init() {
}
#endif // defined(LMIC_PRINTF_TO)

void hal_spi_init();
void hal_init (void * /* bootarg */) {
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
#ifdef CFG_DEBUG
    hal_debug_init();
#endif
}

void hal_failed () {
    // keep IRQs enabled, to allow e.g. USB to continue to run and allow
    // firmware uploads on boards with native USB.
    exit(1);
}

void hal_reboot (void) {
    // TODO
    hal_failed();
}

u1_t hal_getBattLevel (void) {
    // Not implemented
    return 0;
}

void hal_setBattLevel (u1_t /* level */) {
    // Not implemented
}

void hal_fwinfo (hal_fwi* /* fwi */) {
    // Not implemented
}

u1_t* hal_joineui (void) {
    return nullptr;
}

u1_t* hal_deveui (void) {
    return nullptr;
}

u1_t* hal_nwkkey (void) {
    return nullptr;
}

u1_t* hal_appkey (void) {
    return nullptr;
}

u1_t* hal_serial (void) {
    return nullptr;
}

u4_t  hal_region (void) {
    return 0;
}

u4_t  hal_hwid (void) {
    return 0;
}

u4_t  hal_unique (void) {
    return 0;
}

u4_t hal_dnonce_next (void) {
    return os_getRndU2();
}

void onLmicEvent (ev_t ev) {
    printf("%ld: ", os_getTime());
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            printf("EV_SCAN_TIMEOUT\n");
            break;
        case EV_BEACON_FOUND:
            printf("EV_BEACON_FOUND\n");
            break;
        case EV_BEACON_MISSED:
            printf("EV_BEACON_MISSED\n");
            break;
        case EV_BEACON_TRACKED:
            printf("EV_BEACON_TRACKED\n");
            break;
        case EV_JOINING:
            printf("EV_JOINING\n");
            break;
        case EV_JOINED:
            printf("EV_JOINED\n");
            break;
        case EV_RFU1:
            printf("EV_RFU1\n");
            break;
        case EV_JOIN_FAILED:
            printf("EV_JOIN_FAILED\n");
            break;
        case EV_REJOIN_FAILED:
            printf("EV_REJOIN_FAILED\n");
            break;
        case EV_TXCOMPLETE:
            printf("EV_TXCOMPLETE (includes waiting for RX windows)\n");
            if (LMIC.txrxFlags & TXRX_ACK)
              printf("Received ack\n");
            if (LMIC.dataLen)
            {
              uint8_t fPort = LMIC.frame[LMIC.dataBeg -1];              
              printf("Received %d bytes of payload for port %d\n", LMIC.dataLen,(int)fPort);
              if(fPort!=0)
              {
                if(tflora==nullptr)
                {
                  printf("tflora empty\n");
                  break;
                }
                uint8_t *data=LMIC.frame + LMIC.dataBeg;
                tflora->processDownlink(data,LMIC.dataLen);
              }
            }
            break;
        case EV_LOST_TSYNC:
            printf("EV_LOST_TSYNC\n");
            break;
        case EV_RESET:
            printf("EV_RESET\n");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            printf("EV_RXCOMPLETE\n");
            break;
        case EV_LINK_DEAD:
            printf("EV_LINK_DEAD\n");
            break;
        case EV_LINK_ALIVE:
            printf("EV_LINK_ALIVE\n");
            break;
        case EV_SCAN_FOUND:
            printf("EV_SCAN_FOUND\n");
            break;
        case EV_TXSTART:
            printf("EV_TXSTART\n");
            break;
        case EV_TXDONE:
            //printf("EV_TXDONE\n");
            break;
        case EV_DATARATE:
            printf("EV_DATARATE\n");
            break;
        case EV_START_SCAN:
            printf("EV_START_SCAN\n");
            break;
        case EV_ADR_BACKOFF:
            printf("EV_ADR_BACKOFF\n");
            break;
         default:
            printf("Unknown event: %d\n", ev);
            break;
    }
}
