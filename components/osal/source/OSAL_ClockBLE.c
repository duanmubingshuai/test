/**************************************************************************************************
  Filename:       OSAL_ClockBLE.c
  Revised:         
  Revision:        

  Description:    OSAL Clock definition and manipulation functions for BLE projects.

  
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */


#include "comdef.h"
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "timer.h"

/*********************************************************************
 * MACROS
 */

#define	YearLength(yr)	(IsLeapYear(yr) ? 366 : 365)

/*********************************************************************
 * CONSTANTS
 */

// (MAXCALCTICKS * 5) + (max remainder) must be <= (uint16 max),
// so: (13105 * 5) + 7 <= 65535
#define MAXCALCTICKS  ((uint16)(13105))

#define	BEGYEAR	        2000     // UTC started at 00:00:00 January 1, 2000

#define	DAY             86400UL  // 24 hours * 60 minutes * 60 seconds

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
static uint32 previousLLTimerTick = 0;
static uint16 remUsTicks = 0;
static uint16 timeMSec = 0;

// number of seconds since 0 hrs, 0 minutes, 0 seconds, on the
// 1st of January 2000 UTC
UTCTime OSAL_timeSeconds = 0;

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
static uint8 monthLength( uint8 lpyr, uint8 mon );

static void osalClockUpdate( uint16 elapsedMSec );

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalTimeUpdate
 *
 * @brief   Uses the free running rollover count of the MAC backoff timer;
 *          this timer runs freely with a constant 320 usec interval.  The
 *          count of 320-usec ticks is converted to msecs and used to update
 *          the OSAL clock and Timers by invoking osalClockUpdate() and
 *          osalTimerUpdate().  This function is intended to be invoked
 *          from the background, not interrupt level.
 *
 * @param   None.
 *
 * @return  None.
 */
void osalTimeUpdate( void )
{
  uint32 tmp;
  uint16 ticks625us;
  uint16 elapsedMSec = 0;

  // Get the free-running count of 625us timer ticks
  tmp = getMcuPrecisionCount();

  if ( tmp != previousLLTimerTick )
  {
    // Calculate the elapsed ticks of the free-running timer.
    if (tmp > previousLLTimerTick)
        ticks625us = (uint16)(tmp - previousLLTimerTick);
    else 
        ticks625us = (uint16)(0x100000000 - previousLLTimerTick + tmp);

    // Store the LL Timer tick count for the next time through this function.
    previousLLTimerTick = tmp;

    /* It is necessary to loop to convert the usecs to msecs in increments so as
     * not to overflow the 16-bit variables.
     */
    while ( ticks625us > MAXCALCTICKS )
    {
      ticks625us -= MAXCALCTICKS;
      elapsedMSec += MAXCALCTICKS * 5 / 8;
      remUsTicks += MAXCALCTICKS * 5 % 8;
    }

    // update converted number with remaining ticks from loop and the
    // accumulated remainder from loop
    tmp = (ticks625us * 5) + remUsTicks;

    // Convert the 625 us ticks into milliseconds and a remainder
    elapsedMSec += tmp / 8;
    remUsTicks = tmp % 8;

    // Update OSAL Clock and Timers
    if ( elapsedMSec )
    {
      osalClockUpdate( elapsedMSec );
      osalTimerUpdate( elapsedMSec );
    }
  }
}

// change Ti osalTimeUpdate() to 32bit version
// 暂时没有考虑tick越界问题, 32bit，625us时钟tick，越界时间大概是31天
// TODO: 如果要用这个函数，需要做补充单元测试
void osalTimeUpdate1( void )
{
  uint32 tmp;
  uint32 ticks625us;       
  uint16 elapsedMSec = 0;

  // Get the free-running count of 625us timer ticks
  tmp = getMcuPrecisionCount();

  if ( tmp != previousLLTimerTick )
  {
    // Calculate the elapsed ticks of the free-running timer.
    ticks625us = tmp - previousLLTimerTick + remUsTicks;

    // Store the LL Timer tick count for the next time through this function.
    previousLLTimerTick = tmp;
		
    tmp = ticks625us * 5;
    elapsedMSec = (uint16)((tmp >> 3) & 0xffff);
    remUsTicks = (uint16)(tmp - ((uint32)elapsedMSec << 3));

    /*elapsedMSec = 	ticks625us * 5 / 8;
    remUsTicks = elapsedMSec - elapsedMSec * 8 / 5;*/
		
    // Update OSAL Clock and Timers
    if ( elapsedMSec )
    {
      osalClockUpdate( elapsedMSec );
      osalTimerUpdate( elapsedMSec );
    }
  }
}

/*********************************************************************
 * @fn      osalClockUpdate
 *
 * @brief   Updates the OSAL Clock time with elapsed milliseconds.
 *
 * @param   elapsedMSec - elapsed milliseconds
 *
 * @return  none
 */
static void osalClockUpdate( uint16 elapsedMSec )
{
  // Add elapsed milliseconds to the saved millisecond portion of time
  timeMSec += elapsedMSec;

  // Roll up milliseconds to the number of seconds
  if ( timeMSec >= 1000 )
  {
    OSAL_timeSeconds += timeMSec / 1000;
    timeMSec = timeMSec % 1000;
  }
}

/*********************************************************************
 * @fn      osal_setClock
 *
 * @brief   Set the new time.  This will only set the seconds portion
 *          of time and doesn't change the factional second counter.
 *
 * @param   newTime - number of seconds since 0 hrs, 0 minutes,
 *                    0 seconds, on the 1st of January 2000 UTC
 *
 * @return  none
 */
void osal_setClock( UTCTime newTime )
{
  OSAL_timeSeconds = newTime;
}

/*********************************************************************
 * @fn      osal_getClock
 *
 * @brief   Gets the current time.  This will only return the seconds
 *          portion of time and doesn't include the factional second
 *          counter.
 *
 * @param   none
 *
 * @return  number of seconds since 0 hrs, 0 minutes, 0 seconds,
 *          on the 1st of January 2000 UTC
 */
UTCTime osal_getClock( void )
{
  return ( OSAL_timeSeconds );
}

/*********************************************************************
 * @fn      osal_ConvertUTCTime
 *
 * @brief   Converts UTCTime to UTCTimeStruct
 *
 * @param   tm - pointer to breakdown struct
 *
 * @param   secTime - number of seconds since 0 hrs, 0 minutes,
 *          0 seconds, on the 1st of January 2000 UTC
 *
 * @return  none
 */
void osal_ConvertUTCTime( UTCTimeStruct *tm, UTCTime secTime )
{
  // calculate the time less than a day - hours, minutes, seconds
  {
    uint32 day = secTime % DAY;
    tm->seconds = day % 60UL;
    tm->minutes = (day % 3600UL) / 60UL;
    tm->hour = day / 3600UL;
  }

  // Fill in the calendar - day, month, year
  {
    uint16 numDays = secTime / DAY;
    tm->year = BEGYEAR;
    while ( numDays >= YearLength( tm->year ) )
    {
      numDays -= YearLength( tm->year );
      tm->year++;
    }

    tm->month = 0;
    while ( numDays >= monthLength( IsLeapYear( tm->year ), tm->month ) )
    {
      numDays -= monthLength( IsLeapYear( tm->year ), tm->month );
      tm->month++;
    }

    tm->day = numDays;
  }
}

/*********************************************************************
 * @fn      monthLength
 *
 * @param   lpyr - 1 for leap year, 0 if not
 *
 * @param   mon - 0 - 11 (jan - dec)
 *
 * @return  number of days in specified month
 */
static uint8 monthLength( uint8 lpyr, uint8 mon )
{
  uint8 days = 31;

  if ( mon == 1 ) // feb
  {
    days = ( 28 + lpyr );
  }
  else
  {
    if ( mon > 6 ) // aug-dec
    {
      mon--;
    }

    if ( mon & 1 )
    {
      days = 30;
    }
  }

  return ( days );
}

/*********************************************************************
 * @fn      osal_ConvertUTCSecs
 *
 * @brief   Converts a UTCTimeStruct to UTCTime
 *
 * @param   tm - pointer to provided struct
 *
 * @return  number of seconds since 00:00:00 on 01/01/2000 (UTC)
 */
UTCTime osal_ConvertUTCSecs( UTCTimeStruct *tm )
{
  uint32 seconds;

  /* Seconds for the partial day */
  seconds = (((tm->hour * 60UL) + tm->minutes) * 60UL) + tm->seconds;

  /* Account for previous complete days */
  {
    /* Start with complete days in current month */
    uint16 days = tm->day;

    /* Next, complete months in current year */
    {
      int8 month = tm->month;
      while ( --month >= 0 )
      {
        days += monthLength( IsLeapYear( tm->year ), month );
      }
    }

    /* Next, complete years before current year */
    {
      uint16 year = tm->year;
      while ( --year >= BEGYEAR )
      {
        days += YearLength( year );
      }
    }

    /* Add total seconds before partial day */
    seconds += (days * DAY);
  }

  return ( seconds );
}
