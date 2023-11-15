#ifndef MILLIS_H_
#define MILLIS_H_

#include <stdint.h>
#include "millis.h"


uint32_t millis(void);

void ContinuousTimerHandler(void);

void ContinuousTimer(void);


#endif /* MILLIS_H_ */
