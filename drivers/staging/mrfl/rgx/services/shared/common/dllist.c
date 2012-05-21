									     /**************************************************************************//*!
									        @File           dllist.c
									        @Title          Services implementation of double linked lists
									        @Date
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Implements a double linked list
    *//***************************************************************************/

#include "img_types.h"
#include "dllist.h"

IMG_VOID dllist_init(PDLLIST_NODE psListHead)
{
	psListHead->psPrevNode = psListHead;
	psListHead->psNextNode = psListHead;
}

IMG_BOOL dllist_is_empty(PDLLIST_NODE psListHead)
{
	return ((psListHead->psPrevNode == psListHead)
		&& (psListHead->psNextNode == psListHead));
}

IMG_VOID dllist_add_to_tail(PDLLIST_NODE psListHead, PDLLIST_NODE psNewNode)
{
	PDLLIST_NODE psTmp;

	psTmp = psListHead->psPrevNode;

	psListHead->psPrevNode = psNewNode;
	psNewNode->psPrevNode = psTmp;

	psTmp->psNextNode = psNewNode;
	psNewNode->psNextNode = psListHead;
}

IMG_BOOL dllist_node_is_in_list(PDLLIST_NODE psNode)
{
	return (psNode->psNextNode == 0) ? IMG_FALSE : IMG_TRUE;
}

PDLLIST_NODE dllist_get_next_node(PDLLIST_NODE psListHead)
{
	if (psListHead->psNextNode == psListHead) {
		return IMG_NULL;
	} else {
		return psListHead->psNextNode;
	}
}

IMG_VOID dllist_remove_next_node(PDLLIST_NODE psListHead)
{
	PDLLIST_NODE psTmp;

	psTmp = psListHead->psNextNode;

	psListHead->psNextNode = psTmp->psNextNode;
	psTmp->psNextNode->psPrevNode = psListHead;
}

IMG_VOID dllist_move_next_node_to_tail(PDLLIST_NODE psListHead)
{
	PDLLIST_NODE psTmp;

	psTmp = dllist_get_next_node(psListHead);
	dllist_remove_next_node(psListHead);
	dllist_add_to_tail(psListHead, psTmp);
}

IMG_VOID dllist_remove_node(PDLLIST_NODE psListNode)
{
	psListNode->psNextNode->psPrevNode = psListNode->psPrevNode;
	psListNode->psPrevNode->psNextNode = psListNode->psNextNode;

	/* Clear the node to show it's not on a list */
	psListNode->psPrevNode = 0;
	psListNode->psNextNode = 0;
}

/* Walk through all the nodes on the list until the end or a callback returnes FALSE */
IMG_VOID dllist_foreach_node(PDLLIST_NODE psListHead,
			     PFN_NODE_CALLBACK pfnCallBack,
			     IMG_PVOID pvCallbackData)
{
	PDLLIST_NODE psWalker = psListHead->psNextNode;
	PDLLIST_NODE psNextWalker;

	while (psWalker != psListHead) {
		/*
		   The callback function could remove itself from the list
		   so to avoid NULL pointer deference save the next node pointer
		   before calling the callback
		 */
		psNextWalker = psWalker->psNextNode;
		if (pfnCallBack(psWalker, pvCallbackData)) {
			psWalker = psNextWalker;
		} else {
			break;
		}
	}
}
