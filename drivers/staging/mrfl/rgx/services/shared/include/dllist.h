									     /**************************************************************************//*!
									        @File           dllist.h
									        @Title          Double linked list header
									        @Date
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Double linked list interface
    *//***************************************************************************/

#include "img_types.h"

#ifndef _DLLIST_
#define _DLLIST_

/*!
	Pointer to a linked list node
*/
typedef struct _DLLIST_NODE_ *PDLLIST_NODE;

/*!
	Node in a linked list
*/
typedef struct _DLLIST_NODE_ {
	struct _DLLIST_NODE_ *psPrevNode;
	struct _DLLIST_NODE_ *psNextNode;
} DLLIST_NODE;

/*!
	Static initialiser
*/
#define DECLARE_DLLIST(n) \
DLLIST_NODE n = {&n, &n}

									    /*************************************************************************//*!
									       @Function       dllist_init

									       @Description    Initialize a new double linked list

									       @Input          psListHead              List head Node

									     */
/*****************************************************************************/
IMG_INTERNAL IMG_VOID dllist_init(PDLLIST_NODE psListHead);

									    /*************************************************************************//*!
									       @Function       dllist_is_empty

									       @Description    Returns whether the list is empty

									       @Input          psListHead              List head Node

									     */
/*****************************************************************************/
IMG_INTERNAL IMG_BOOL dllist_is_empty(PDLLIST_NODE psListHead);

									    /*************************************************************************//*!
									       @Function       dllist_add_to_tail

									       @Description    Add psNewNode to tail of list psListHead

									       @Input          psListHead             Head Node

									     */
/*****************************************************************************/
IMG_INTERNAL
    IMG_VOID dllist_add_to_tail(PDLLIST_NODE psListHead,
				PDLLIST_NODE psNewNode);

									    /*************************************************************************//*!
									       @Function       dllist_node_is_in_list

									       @Description    Returns IMG_TRUE if psNode is in a list 

									       @Input          psNode             List node

									     */
/*****************************************************************************/
IMG_INTERNAL IMG_BOOL dllist_node_is_in_list(PDLLIST_NODE psNode);

									    /*************************************************************************//*!
									       @Function       dllist_get_next_node

									       @Description    Returns the list node after psListHead or IMG_NULL psListHead
									       is the only element in the list.

									       @Input          psListHead             List node to start the operation

									     */
/*****************************************************************************/
IMG_INTERNAL PDLLIST_NODE dllist_get_next_node(PDLLIST_NODE psListHead);

									    /*************************************************************************//*!
									       @Function       dllist_remove_next_node

									       @Description    Remove the list node after psListHead

									       @Input          psListHead             List node to start the operation

									     */
/*****************************************************************************/
IMG_INTERNAL IMG_VOID dllist_remove_next_node(PDLLIST_NODE psListHead);

									    /*************************************************************************//*!
									       @Function       dllist_move_next_node_to_tail

									       @Description    Move the node after psListHead to the tail of the list

									       @Input          psListHead             List node to start the operation

									     */
/*****************************************************************************/
IMG_INTERNAL IMG_VOID dllist_move_next_node_to_tail(PDLLIST_NODE psListHead);

									    /*************************************************************************//*!
									       @Function       dllist_remove_node

									       @Description    Removes psListNode from the list where it currently belongs

									       @Input          psListNode             List node to be removed

									     */
/*****************************************************************************/
IMG_INTERNAL IMG_VOID dllist_remove_node(PDLLIST_NODE psListNode);

/*!
	Callback function called on each element of the list
*/
typedef IMG_BOOL(*PFN_NODE_CALLBACK) (PDLLIST_NODE psNode,
				      IMG_PVOID pvCallbackData);

									    /*************************************************************************//*!
									       @Function       dllist_foreach_node

									       @Description    Walk through all the nodes on the list until the 
									       end or a callback returns FALSE

									       @Input          psListHead                       List node to start the operation
									       @Input                   pfnCallBack                     PFN_NODE_CALLBACK function called on each element       
									       @Input                   pvCallbackData          Data passed to pfnCallBack alongside the current Node

									     */
/*****************************************************************************/
IMG_INTERNAL
    IMG_VOID dllist_foreach_node(PDLLIST_NODE psListHead,
				 PFN_NODE_CALLBACK pfnCallBack,
				 IMG_PVOID pvCallbackData);
#endif				/* _DLLIST_ */
