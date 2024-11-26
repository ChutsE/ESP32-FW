#ifndef RTC_H
#define RTC_H

    #include <stdint.h>
    int64_t get_rtc_time(void);
    void delay_us(unsigned int microseconds);
    void delay_ms(unsigned int ms);

#endif 