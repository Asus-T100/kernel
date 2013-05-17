#ifndef __HMEM_PUBLIC_H_INCLUDED__
#define __HMEM_PUBLIC_H_INCLUDED__

#include <stddef.h>		/* size_t */

/*! Return the size of HMEM[ID]
 
 \param	ID[in]				HMEM identifier

 \Note: The size is the byte size of the area it occupies
		in the address map. I.e. disregarding internal structure

 \return sizeof(HMEM[ID])
 */
STORAGE_CLASS_HMEM_H size_t sizeof_hmem(
	const hmem_ID_t		ID);

#endif /* __HMEM_PUBLIC_H_INCLUDED__ */
