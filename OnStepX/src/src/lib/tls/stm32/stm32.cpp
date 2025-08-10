#include <STM32RTC.h>
#include "stm32.h"

#if defined(TIME_LOCATION_SOURCE) && TIME_LOCATION_SOURCE == STM32 || \
    (defined(TIME_LOCATION_SOURCE_FALLBACK) && TIME_LOCATION_SOURCE_FALLBACK == STM32)

/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();

#include <TimeLib.h> 
//#include "../PPS.h"

bool TlsStm32::init() {
    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    rtc.begin(); // initialize RTC 24H format
    ready = true;
    return ready;
}

void TlsStm32::set(JulianDate ut1) {
  if (!ready) return;

  GregorianDate greg = calendars.julianDayToGregorian(ut1);

  double f1 = fabs(ut1.hour) + TLS_CLOCK_SKEW;
  int h = floor(f1);
  double m = (f1 - h)*60.0;
  double s = (m - floor(m))*60.0;

  set(greg.year, greg.month, greg.day, h, floor(m), floor(s));
}

void TlsStm32::set(int year, int month, int day, int hour, int minute, int second) {
    setTime(hour, minute, second, day, month, year);
    rtc.setTime(hour, minute, second);
    rtc.setDate(day, month, year-2000); // STM32 RTC uses 2000 as base year

    }

bool TlsStm32::get(JulianDate &ut1) {
  if (!ready) return false;

  //unsigned long rtcTime = rtc.getEpoch(); // get time from Teensy RTC
  //setTime(rtcTime);                           // set system time
  int rtcHours = rtc.getHours();
  int rtcMinutes = rtc.getMinutes();
  int rtcSeconds = rtc.getSeconds();
  int rtcDay = rtc.getDay();
  int rtcMonth = rtc.getMonth();
  int rtcYear = rtc.getYear()+2000; // STM32 RTC uses 2000 as base year

  setTime(rtcHours, rtcMinutes, rtcSeconds, rtcDay, rtcMonth, rtcYear);
  if (rtcYear >= 0 && rtcYear <= 3000 && rtcMonth >= 1 && rtcMonth <= 12 && rtcDay >= 1 && rtcDay <= 31 &&
      rtcHours <= 23 && rtcMinutes <= 59 && rtcSeconds <= 59) {
    GregorianDate greg;
    greg.year = rtcYear; // STM32 RTC uses 2000 as base year
    greg.month = rtcMonth;
    greg.day = rtcDay;
    ut1 = calendars.gregorianToJulianDay(greg);
    ut1.hour = rtcHours + rtcMinutes / 60.0 + rtcSeconds / 3600.0;
  }
  
  //DLF("MSG: STM32RTC Time GET: %04d-%02d-%02d %02d:%02d:%02d\n", rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  return true;
}
#endif