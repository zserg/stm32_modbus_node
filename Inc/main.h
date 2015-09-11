#define BOARD_MCK               72000000

#define MB_TIMER_DEBUG                      ( 0 )
#define MB_TIMER_PRESCALER                  ( BOARD_MCK-1 )
#define MB_TIMER_TICKS                      ( BOARD_MCK / (MB_TIMER_PRESCALER + 1) )
#define MB_50US_TICKS                       ( 20000UL )

