#include "include/rtc_regs.h"
#include "include/rtc.h"
#include "include/utils.h"

//#include "esp_log.h"
//#define TAG "TIMER"

int64_t get_rtc_time(void) {
    REG_WRITE(RTC_CNTL_TIME_UPDATE_REG, 1 << 31);
    uint32_t lo = REG_READ(RTC_CNTL_TIME0_REG);
    uint32_t hi = REG_READ(RTC_CNTL_TIME1_REG);
    int64_t time_us = ((int64_t)hi << 32) | lo;
    return time_us;
}

void delay_us(unsigned int microseconds){
    int64_t start = get_rtc_time();
    while (get_rtc_time() - start < microseconds){
        //ESP_LOGI(TAG, "Timer value: %llu", get_rtc_time());
    }
}

void delay_ms(unsigned int ms){
    delay_us(ms * 1000);
}

