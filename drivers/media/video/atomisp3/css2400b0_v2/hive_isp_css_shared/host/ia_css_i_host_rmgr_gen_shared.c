#include "ia_css_i_host_rmgr_gen_shared.h"
#ifndef __KERNEL__
#include <stdbool.h>
#endif
#include <assert_support.h>
#include <ia_css.h>

//#include "memory_access.h"

//#include "sh_css_debug.h"


static enum ia_css_err set_resource_pool_size(ia_css_resource_list_t* pool, uint32_t size);
static enum ia_css_err acquire_resource(ia_css_resource_list_t* pool, uint32_t* resource_id);
static enum ia_css_err release_resource(ia_css_resource_list_t* pool, uint32_t resource_id);


/* test functionality*/
#define TEST_RM 
#ifdef TEST_RM
void check_aquire(ia_css_resource_type_t resource_type, uint32_t* resource_id,uint32_t expected, bool shouldfail);
void check_release(ia_css_resource_type_t resource_type, uint32_t resource_id, bool shouldfail);
void Test_resource_manager(void);
#endif


ia_css_resource_list_t g_resource_sp_thread_pool;
ia_css_resource_list_t g_resource_queue_pool;


/*****************************************************************************
In the ia_css_resource_mgr_setup function the resources can be configured 
arguments:
resource_type: can be any enum from the ia_css_resource_type_t list
size: the new resource pool size
 *****************************************************************************/


enum ia_css_err ia_css_i_host_rmgr_init_gen(ia_css_resource_type_t resource_type)
{
	enum ia_css_err rval = IA_CSS_ERR_INTERNAL_ERROR;
	assert((resource_type > IA_CSS_RESOURCE_START) && (resource_type < IA_CSS_RESOURCE_END));

	switch (resource_type)
	{
		case IA_CSS_RESOURCE_SP_THREAD:
		{
			g_resource_sp_thread_pool.size = 0;
		rval = IA_CSS_SUCCESS;
		break;
	}
	case IA_CSS_RESOURCE_HOST2SP_QUEUE:
	{
		g_resource_queue_pool.size = 0;
		rval = IA_CSS_SUCCESS;
		break;
	}
	default:
	{
		assert(0); /* Resource type not implemented please add it to the list. */
		break;
	}
	}
	return rval;
}


enum ia_css_err ia_css_i_host_rmgr_setup_gen(ia_css_resource_type_t resource_type, uint32_t size)
{
	enum ia_css_err rval = IA_CSS_ERR_INTERNAL_ERROR; 
	assert((resource_type > IA_CSS_RESOURCE_START) && (resource_type <	IA_CSS_RESOURCE_END));
	switch (resource_type)
	{
	case IA_CSS_RESOURCE_SP_THREAD:
	{
		return set_resource_pool_size(&g_resource_sp_thread_pool, size);
		break;
	}
	case IA_CSS_RESOURCE_HOST2SP_QUEUE:
	{
		return set_resource_pool_size(&g_resource_queue_pool,  size);
		break;
	}
	default:
	{
		assert(0); /* Resource type not implemented please add it to the list. */
		break;
	}
	}
	return rval;
}



enum ia_css_err ia_css_i_host_rmgr_acq_gen(ia_css_resource_type_t resource_type, uint32_t* resource_id)
{
	enum ia_css_err rval = IA_CSS_ERR_INTERNAL_ERROR; 

	assert((resource_type > IA_CSS_RESOURCE_START) && (resource_type <	IA_CSS_RESOURCE_END));
	switch (resource_type)
	{
		case IA_CSS_RESOURCE_SP_THREAD:
		{
			return acquire_resource(&g_resource_sp_thread_pool, resource_id);
			break;
		}
	case IA_CSS_RESOURCE_HOST2SP_QUEUE:
		{
			return acquire_resource(&g_resource_queue_pool, resource_id);
			break;
		}
	default:
		{
			assert(0);  /* Resource type not implemented please add it to the list. */
			break;
		}
	}
	return rval;
}



enum ia_css_err ia_css_i_host_rmgr_rel_gen(ia_css_resource_type_t resource_type, uint32_t resource_id)
{
	assert((resource_type > IA_CSS_RESOURCE_START) && (resource_type < IA_CSS_RESOURCE_END));
	switch (resource_type)
	{
		case IA_CSS_RESOURCE_SP_THREAD:
		{
		return release_resource(&g_resource_sp_thread_pool, resource_id);
		break;
		}
		case IA_CSS_RESOURCE_HOST2SP_QUEUE:
		{
			return release_resource(&g_resource_queue_pool, resource_id);
			break;
		}
		default:
		{
			assert(0);	/* Resource type not implemented please add it to the list. */
			break;
		}
	}
	return IA_CSS_ERR_INTERNAL_ERROR; 
}




/**************************************************************************************************/
/***********************              Resource manager support functions           *******************************/
/**************************************************************************************************/


/* set the pool size */
static enum ia_css_err 
set_resource_pool_size(ia_css_resource_list_t* pool, uint32_t size)
{
	enum ia_css_err rval = IA_CSS_SUCCESS; 
	uint32_t i = 0;
	if (size > IA_CSS_RESOURCE_LIST_MAX_AMOUNT) 
		return IA_CSS_ERR_RESOURCE_LIST_TO_SMALL; 

	if (size < pool->size)
	{
	/* Size is going to be reduced, check for reserved items */
	for (i = size; i < pool->size; i++)
	{
		if (pool->list[i].reserved == IA_CSS_RESOURCE_RESERVED)
		{
			return IA_CSS_ERR_RESOURCE_ITEMS_STILL_ALLOCATED;
		}
	}
	}
	else
	{
		for (i = pool->size; i <= size; i++)
		{
			pool->list[i].id = i;
		}
	}
	pool->size = size;
return rval;
}



/* request for resource, if available an id will be send back */
static enum ia_css_err 
acquire_resource(ia_css_resource_list_t* pool, uint32_t* resource_id)
{
	enum ia_css_err rval = IA_CSS_ERR_RESOURCE_EXHAUSTED;
	bool free_place_found = false;
	uint16_t i = 0;
	/* loop trough list */
	for (;( i < pool->size); i++)
	{
		if (pool->list[i].reserved == IA_CSS_RESOURCE_AVAILABLE)
		{
			*resource_id = pool->list[i].id;
			pool->list[i].reserved = IA_CSS_RESOURCE_RESERVED;
			rval = IA_CSS_SUCCESS;
			break;
		}
	}
	return rval;
}


/* release resource, if available */
static enum ia_css_err release_resource(ia_css_resource_list_t* pool, uint32_t resource_id)
{
	enum ia_css_err rval = IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	bool resource_found = false;
	uint16_t i = 0;
	/* loop trough list */
	for (;( i <= pool->size) || (resource_found == false); i++)
	{
		if (pool->list[i].id == resource_id)
		{
			resource_found = true;
			if (pool->list[i].reserved != IA_CSS_RESOURCE_RESERVED)
			{
				return	IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;   
			}
			else
			{
				pool->list[i].reserved = IA_CSS_RESOURCE_AVAILABLE;
			}
		rval = IA_CSS_SUCCESS;
		}
	}
return rval;
}




#ifdef TEST_RM

/**************************************************************************************************/
/***********************              Resource manager test functions                *******************************/
/**************************************************************************************************/

void check_aquire(ia_css_resource_type_t resource_type, uint32_t* resource_id,uint32_t expected, bool shouldfail)
{
	enum ia_css_err rval = 0;

	if (!shouldfail)
	{
		rval = ia_css_i_host_rmgr_acq_gen(resource_type, resource_id);
		if(rval !=IA_CSS_SUCCESS) sh_css_dtrace(SH_DBG_ERROR, "Failed rval = %d\n", rval);  
		if (*resource_id != expected)
			while(1);//sh_css_dtrace(SH_DBG_ERROR,"Failed: rval = %d ... expected = %d\n",*resource_id, expected);
		else sh_css_dtrace(SH_DBG_ERROR, "Succes: resourcetype = %x - Resource	%d acquired\n", resource_type, *resource_id);	 
			
		
	}
	else
	{
		if (IA_CSS_ERR_RESOURCE_EXHAUSTED == ia_css_i_host_rmgr_acq_gen(resource_type, resource_id))
			sh_css_dtrace(SH_DBG_ERROR, "Succes: resource exhausted\n"); 
		else while(1);//	sh_css_dtrace(SH_DBG_ERROR, "Failed: error not what expected\n");  
	}
}

void check_release(ia_css_resource_type_t resource_type, uint32_t resource_id, bool shouldfail)
{
	enum ia_css_err rval = 0;

	if (!shouldfail)
	{
		rval = ia_css_i_host_rmgr_rel_gen(resource_type, resource_id);
		if(rval !=IA_CSS_SUCCESS) while(1);//sh_css_dtrace(SH_DBG_ERROR, "Failed rval = %d\n", rval); 
	}
	else
	{
		rval = ia_css_i_host_rmgr_rel_gen(resource_type, resource_id);
		if(rval == IA_CSS_ERR_RESOURCE_NOT_AVAILABLE) sh_css_dtrace(SH_DBG_ERROR, "Success: resource not avaliable\n"); 
	}
}



void test_resource_manager(void)
{
	enum ia_css_err rval = 0;
	uint32_t resource_id_q = 0;
	uint32_t resource_id_thread = 0;
	
	//sh_css_set_dtrace_level(9);
	
	rval = ia_css_i_host_rmgr_init_gen(IA_CSS_RESOURCE_SP_THREAD);
	assert(rval ==IA_CSS_SUCCESS);
	rval = ia_css_i_host_rmgr_init_gen(IA_CSS_RESOURCE_HOST2SP_QUEUE);
	assert(rval ==IA_CSS_SUCCESS);
	
	rval = ia_css_i_host_rmgr_setup_gen(IA_CSS_RESOURCE_SP_THREAD, 10);
	assert(rval ==IA_CSS_SUCCESS);
	rval = ia_css_i_host_rmgr_setup_gen(IA_CSS_RESOURCE_HOST2SP_QUEUE, 1111);
	assert(rval!= IA_CSS_SUCCESS); 
	rval = ia_css_i_host_rmgr_setup_gen(IA_CSS_RESOURCE_HOST2SP_QUEUE, 1);
	assert(rval == IA_CSS_SUCCESS); 

	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 0, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 1, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 2, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 3, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 4, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 5, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 6, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 7, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 8, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 9, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, IA_CSS_RESOURCE_LIST_MAX_AMOUNT, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, IA_CSS_RESOURCE_LIST_MAX_AMOUNT + 1, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, IA_CSS_RESOURCE_LIST_MAX_AMOUNT + 100, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 0, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 1, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 2, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 3, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 4, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 5, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 6, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 7, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 8, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 9, 1);
	check_release(IA_CSS_RESOURCE_SP_THREAD, 3, 0);
	check_release(IA_CSS_RESOURCE_SP_THREAD, 8, 0);
	check_release(IA_CSS_RESOURCE_SP_THREAD, 2, 0);
	check_release(IA_CSS_RESOURCE_SP_THREAD, 7, 0);
	check_release(IA_CSS_RESOURCE_SP_THREAD, 7, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 2, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 3, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 7, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 8, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 8, 1);
	

	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 0, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 1, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 2, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 3, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 4, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 5, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 6, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 7, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 8, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 9, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, IA_CSS_RESOURCE_LIST_MAX_AMOUNT, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, IA_CSS_RESOURCE_LIST_MAX_AMOUNT + 1, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, IA_CSS_RESOURCE_LIST_MAX_AMOUNT + 100, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 0, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 1, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 2, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 3, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 4, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 5, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 6, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 7, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 8, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 9, 1);
	check_release(IA_CSS_RESOURCE_HOST2SP_QUEUE, 3, 0);
	check_release(IA_CSS_RESOURCE_HOST2SP_QUEUE, 8, 0);
	check_release(IA_CSS_RESOURCE_HOST2SP_QUEUE, 2, 0);
	check_release(IA_CSS_RESOURCE_HOST2SP_QUEUE, 7, 0);
	check_release(IA_CSS_RESOURCE_HOST2SP_QUEUE, 7, 1);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 2, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 3, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 7, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 8, 0);
	check_aquire(IA_CSS_RESOURCE_HOST2SP_QUEUE, &resource_id_q, 8, 1);

	check_release(IA_CSS_RESOURCE_SP_THREAD, 1, 0);
	check_release(IA_CSS_RESOURCE_SP_THREAD, 2, 0);
	check_release(IA_CSS_RESOURCE_SP_THREAD, 3, 0);
	check_release(IA_CSS_RESOURCE_SP_THREAD, 4, 0);
	check_release(IA_CSS_RESOURCE_SP_THREAD, 2, 1);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 1, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 2, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 3, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 4, 0);
	check_aquire(IA_CSS_RESOURCE_SP_THREAD, &resource_id_thread, 5, 1);



}
#endif

