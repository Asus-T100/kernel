#include "ia_css_i_host_rmgr_gen_shared.h"

#ifndef __KERNEL__
#include <stdbool.h>
#endif

#include <assert_support.h>
#include <ia_css.h>

//#include "memory_access.h"

//#include "sh_css_debug.h"


#define IA_CSS_RESOURCE_LIST_MAX_AMOUNT 10 
#define IA_CSS_RESOURCE_RESERVED  1
#define IA_CSS_RESOURCE_AVAILABLE 0


#error


ia_css_err_t set_resource_pool_size(ia_css_resource_list_t* pool, uint32_t size);
ia_css_err_t acquire_resource(ia_css_resource_list_t* pool, uint32_t* resource_id);
ia_css_err_t release_resource(ia_css_resource_list_t* pool, uint32_t resource_id);


typedef enum ia_css_resource_type
(
    IA_CSS_RESOURCE_START  = 1230,     // start at random number for code robustness, 
    IA_CSS_RESOURCE_SP_THREAD,        // register the threads of the sp
    IA_CSS_RESOURCE_HOST2SP_QUEUE,    // register the host to sp queues
    IA_CSS_RESOURCE_END               
}ia_css_resource_type_t;

typedef struct ia_css_resource_list_item
{
    uint32_t id;
    uint16_t reserved;
}ia_css_resource_list_item_t

typedef struct ia_css_resource_list
{
    ia_css_resource_list_item_t list[IA_CSS_RESOURCE_LIST_MAX_AMOUNT];
    uint16_t size;
}ia_css_resource_list_t;


ia_css_resource_list_t g_resource_sp_thread_pool;
ia_css_resource_list_t g_resource_queue_pool;


/*****************************************************************************
 In the ia_css_resource_mgr_setup function the resources can be configured 
  arguments:                                                                
  resource_type: can be any enum from the ia_css_resource_type_t list       
  size: the new resource pool size                                          
 *****************************************************************************/

ia_css_i_host_rmgr_init_gen

ia_css_err_t ia_css_i_host_rmgr_init_gen(ia_css_resource_type_t resource_type)
{
    ia_css_err_t rval = IA_CSS_ERR_INTERNAL_ERROR; 
    assert((resource_type > IA_CSS_RESOURCE_START) && (resource_type <  IA_CSS_RESOURCE_END));
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

ia_css_i_host_rmgr_setup_gen

ia_css_err_t ia_css_i_host_rmgr_setup_gen(ia_css_resource_type_t resource_type, uint32_t size)
{
    ia_css_err_t rval = IA_CSS_ERR_INTERNAL_ERROR; 
    assert((resource_type > IA_CSS_RESOURCE_START) && (resource_type <  IA_CSS_RESOURCE_END));
    switch (resource_type)
    {
        case IA_CSS_RESOURCE_SP_THREAD:
        {
           return set_resource_pool_size(&g_resource_sp_thread_pool, uint32_t size);
           break;
        }
        case IA_CSS_RESOURCE_HOST2SP_QUEUE:
        {
            return set_resource_pool_size(&g_resource_queue_pool, uint32_t size);
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



ia_css_err_t ia_css_i_host_rmgr_acq_gen(ia_css_resource_type_t resource_type, void* id)
{
    ia_css_err_t rval = IA_CSS_ERR_INTERNAL_ERROR; 

    assert((resource_type > IA_CSS_RESOURCE_START) && (resource_type <  IA_CSS_RESOURCE_END));
    switch (resource_type)
    {
       case IA_CSS_RESOURCE_SP_THREAD:
       {
          return acquire_resource(&g_resource_sp_thread_pool, id)
          break;
       }
       case IA_CSS_RESOURCE_HOST2SP_QUEUE:
       {
          return acquire_resource(&g_resource_queue_pool, id);
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



void* ia_css_i_host_rmgr_rel_gen(ia_css_resource_type resource_type, uint32_t size)
{
    assert((resource_type > IA_CSS_RESOURCE_START) && (resource_type <  IA_CSS_RESOURCE_END));
    switch (resource_type)
    {
        case IA_CSS_RESOURCE_SP_THREAD:
        {
            return release_resource(&g_resource_sp_thread_pool, id);
            break;
        }
        case IA_CSS_RESOURCE_HOST2SP_QUEUE:
        {
            return release_resource(&g_resource_queue_pool, id);
            break;
        }
        default:
        {
            assert(0);  /* Resource type not implemented please add it to the list. */
            break;
        }
    }
}




/**************************************************************************************************/
/***********************          Resource   manager support functions             *******************************/
/**************************************************************************************************/


/* set the pool size */
static ia_css_err_t set_resource_pool_size(ia_css_resource_list_t* pool, uint32_t size)
{
    ia_css_err_t rval = IA_CSS_SUCCESS; 
    uint32_t i = 0;
    if (size > IA_CSS_RESOURCE_LIST_MAX_AMOUNT) return IA_CSS_ERR_RESOURCE_LIST_TO_SMALL; 

    if (size < pool->size)
    {   
        /* Size is going to be reduced, check for reserved items */
        for (i = size; i <= pool->size; i++)
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
static ia_css_err_t acquire_resource(ia_css_resource_list_t* pool, uint32_t* resource_id)
{
    ia_css_err_t rval = IA_CSS_ERR_RESOURCE_EXHAUSTED;
    bool free_place_found = false;
    uint16_t i = 0;
    /* loop trough list */
    for (;( i <= pool->size) || (free_place_found == false); i++)
    {
        if (pool->list[i].reserved == IA_CSS_RESOURCE_AVAILABLE)
        {
            free_place_found = true;
            resource_id = pool->list[i].id;
            pool->list[i].reserved = IA_CSS_RESOURCE_RESERVED;
            rval = IA_CSS_SUCCESS;
        }
    }
    
    return rval;
}


/* release resource, if available */
static ia_css_err_t release_resource(ia_css_resource_list_t* pool, uint32_t resource_id)
{
    ia_css_err_t rval = IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
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
                return  IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;   
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









