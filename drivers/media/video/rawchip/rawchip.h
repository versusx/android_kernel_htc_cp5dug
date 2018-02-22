#ifndef __RAWCHIP__
#define __RAWCHIP__

#include "sensor.h"

/*===========================================================================
 * FUNCTION    - x -  
 *
 * DESCRIPTION:
 *==========================================================================*/

 typedef	struct
{
	uint8_t 	bXStart;
	uint8_t 	bYStart;
	uint8_t 	bXEnd;
	uint8_t 	bYEnd;
}Yushan_AF_ROI_t;
struct msm_AFSU_info {
	Yushan_AF_ROI_t sYushanAfRoi[5];
	uint8_t active_number;
};	
struct msm_AFSU_info afsu_info;

void run_afsu(void);
void stop_afsu(void);
void get_afsu(void);

	

#endif /* __RAWCHIP__ */
