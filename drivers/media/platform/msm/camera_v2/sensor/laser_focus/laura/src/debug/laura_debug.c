/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#include "laura_debug.h"
#include "show_log.h"

/** @brief laura read range debug
*	
*	@param vfile virtual file which provide information to the user or system administrator
*	@param DMax the farthest detection distance in current environment
*	@param errorStatus the error status
*	@param record_in_file record debug information to file
*
*/
int laura_debug_register_dump(struct seq_file *vfile, int DMax, int errorStatus, bool record_in_file){
	struct msm_laser_focus_ctrl_t *dev_t = get_laura_ctrl();

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	mutex_ctrl(dev_t, MUTEX_LOCK);
	
	if (dev_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		dev_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, dev_t->device_state);
		mutex_ctrl(dev_t, MUTEX_UNLOCK);
		return -EBUSY;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

/** @brief Dump register value 
*	
*	@param vfile virtual file which provide information to the user or system administrator
*	@param v
*
*/
int dump_laura_register_read(struct seq_file *vfile, void *v){
	int rc = 0;
	struct msm_laser_focus_ctrl_t *dev_t = get_laura_ctrl();

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	mutex_ctrl(dev_t, MUTEX_LOCK);
	
	/* Dump register value */
	rc = dump_register(vfile, dev_t, DUMP_REGISTER_RANGE_MIN, DUMP_REGISTER_RANGE_MAX);
	
	mutex_ctrl(dev_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}
