#ifndef ALS_DRV_H_
#define ALS_DRV_H_

#include <stdint.h>

typedef struct ambient_light
{
	uint16_t lux;
	uint16_t full_spectrum;
	uint16_t infrared;
} als_data_t;

int16_t als_init(void);
int16_t als_get_data(als_data_t *als);

#endif /* ALS_DRV_H_ */