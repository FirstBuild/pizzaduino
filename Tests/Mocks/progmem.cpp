
#include <avr/pgmspace.h>

uint16_t pgm_read_word(const uint16_t *pWord)
{
   return *pWord;
}
