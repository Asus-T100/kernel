/****************************************************************
 *
 * Time   : 2012-09-12, 19:22.
 * Author : zhengjie.lu@intel.com
 * Comment:
 * - Define the software event IDs.
 *
 * Time   : 2012-09-06, 11:16.
 * Author : zhengjie.lu@intel.com
 * Comment:
 * - Initial version.
 *
 ****************************************************************/

#ifndef __SW_EVENT_GLOBAL_H_INCLUDED__
#define __SW_EVENT_GLOBAL_H_INCLUDED__

#define MAX_NR_OF_PAYLOADS_PER_SW_EVENT 4

#define SP_SW_EVENT_ID_0	0	/* for the error		*/
#define SP_SW_EVENT_ID_1	1	/* for the host2sp_buffer_queue */
#define SP_SW_EVENT_ID_2	2	/* for the sp2host_buffer_queue */
#define SP_SW_EVENT_ID_3	3	/* for the sp2host_event_queue  */
#define SP_SW_EVENT_ID_4	4	/* for the start stream cmd */
#define SP_SW_EVENT_ID_5	5	/* for the stop stream cmd  */

/*********************************************
 *
 * Hack for Baytrail.
 *
 * AUTHOR: zhengjie.lu@intel.com
 * TIME: 2013-01-19, 14:38.
 * LOCATION: Santa Clara, U.S.A.
 * COMMENT:
 * Define a new Host2SP event which indicates
 * the Host has passed the pointers of the
 * empty MIPI buffers to the SP.
 *
 ********************************************/	
#define SP_SW_EVENT_ID_6	6	/* for the completion of passing the
					   pointers of the empty MIPI buffers
					   from the Host to the SP
					*/
/** End of hack for Baytrail **/

#endif /* __SW_EVENT_GLOBAL_H_INCLUDED__ */

