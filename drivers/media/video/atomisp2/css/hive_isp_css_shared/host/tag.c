#include "tag.h"

#ifndef __INLINE_QUEUE__
#include "queue_private.h"
#endif /* __INLINE_QUEUE__ */

/**
 * @brief	Creates the tag description from the given parameters.
 * @param[in]	num_captures
 * @param[in]	skip
 * @param[in]	offset
 * @param[out]	tag_descr
 */
void
sh_css_create_tag_descr(int num_captures,
			unsigned int skip,
			int offset,
			unsigned int exp_id,
			struct sh_css_tag_descr *tag_descr)
{
	tag_descr->num_captures = num_captures;
	tag_descr->skip		= skip;
	tag_descr->offset	= offset;
	tag_descr->exp_id	= exp_id;
}

/**
 * @brief	Encodes the members of tag description into a 32-bit value.
 * @param[in]	tag		Pointer to the tag description
 * @return	(unsigned int)	Encoded 32-bit tag-info
 */
unsigned int
sh_css_encode_tag_descr(struct sh_css_tag_descr *tag)
{
	unsigned int num_captures = tag->num_captures;
	unsigned int skip = tag->skip;
	int offset = tag->offset;
	unsigned int exp_id = tag->exp_id;

	unsigned int encoded_tag = (num_captures & 0x00000FFF)
				|  (exp_id       & 0x0000000F) << 12
				|  (skip         & 0x000000FF) << 16
				|  (offset       & 0x000000FF) << 24;

	return encoded_tag;
}

