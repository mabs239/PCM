#include <avr/pgmspace.h>

#ifdef __cplusplus
extern "C" {
#endif

void startPlayback(unsigned char const *data, int length);
void stopPlayback();

#ifdef __cplusplus
}
#endif
