/* mpicoder.c  -  Coder for the external representation of MPIs
 * Copyright (C) 1998, 1999 Free Software Foundation, Inc.
 *
 * This file is part of GnuPG.
 *
 * GnuPG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * GnuPG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA
 */

#include "mpi-internal.h"

#define DIM(v) (sizeof(v)/sizeof((v)[0]))
#define MAX_EXTERN_MPI_BITS 16384

static uint8_t asn[15] =	/* Object ID is 1.3.14.3.2.26 */
{ 0x30, 0x21, 0x30, 0x09, 0x06, 0x05, 0x2b, 0x0e, 0x03,
	0x02, 0x1a, 0x05, 0x00, 0x04, 0x14
};

MPI do_encode_md(const void *sha_buffer, unsigned nbits)
{
	int nframe = (nbits + 7) / 8;
	uint8_t *frame, *fr_pt;
	int i = 0, n;
	size_t asnlen = DIM(asn);
	MPI a = MPI_NULL;

	if (SHA1_DIGEST_LENGTH + asnlen + 4 > nframe)
		pr_info("MPI: can't encode a %d bit MD into a %d bits frame\n",
		       (int)(SHA1_DIGEST_LENGTH * 8), (int)nbits);

	/* We encode the MD in this way:
	 *
	 *       0  A PAD(n bytes)   0  ASN(asnlen bytes)  MD(len bytes)
	 *
	 * PAD consists of FF bytes.
	 */
	frame = kmalloc(nframe, GFP_KERNEL);
	if (!frame)
		return MPI_NULL;
	n = 0;
	frame[n++] = 0;
	frame[n++] = 1;		/* block type */
	i = nframe - SHA1_DIGEST_LENGTH - asnlen - 3;

	if (i <= 1) {
		pr_info("MPI: message digest encoding failed\n");
		kfree(frame);
		return a;
	}

	memset(frame + n, 0xff, i);
	n += i;
	frame[n++] = 0;
	memcpy(frame + n, &asn, asnlen);
	n += asnlen;
	memcpy(frame + n, sha_buffer, SHA1_DIGEST_LENGTH);
	n += SHA1_DIGEST_LENGTH;

	i = nframe;
	fr_pt = frame;

	if (n != nframe) {
		printk
		    ("MPI: message digest encoding failed, frame length is wrong\n");
		kfree(frame);
		return a;
	}

	a = mpi_alloc((nframe + BYTES_PER_MPI_LIMB - 1) / BYTES_PER_MPI_LIMB);
	mpi_set_buffer(a, frame, nframe, 0);
	kfree(frame);

	return a;
}

MPI mpi_read_from_buffer(const void *xbuffer, unsigned *ret_nread)
{
	const uint8_t *buffer = xbuffer;
	int i, j;
	unsigned nbits, nbytes, nlimbs, nread = 0;
	mpi_limb_t a;
	MPI val = MPI_NULL;

	if (*ret_nread < 2)
		goto leave;
	nbits = buffer[0] << 8 | buffer[1];

	if (nbits > MAX_EXTERN_MPI_BITS) {
		pr_info("MPI: mpi too large (%u bits)\n", nbits);
		goto leave;
	}
	buffer += 2;
	nread = 2;

	nbytes = (nbits + 7) / 8;
	nlimbs = (nbytes + BYTES_PER_MPI_LIMB - 1) / BYTES_PER_MPI_LIMB;
	val = mpi_alloc(nlimbs);
	if (!val)
		return MPI_NULL;
	i = BYTES_PER_MPI_LIMB - nbytes % BYTES_PER_MPI_LIMB;
	i %= BYTES_PER_MPI_LIMB;
	val->nbits = nbits;
	j = val->nlimbs = nlimbs;
	val->sign = 0;
	for (; j > 0; j--) {
		a = 0;
		for (; i < BYTES_PER_MPI_LIMB; i++) {
			if (++nread > *ret_nread) {
				printk
				    ("MPI: mpi larger than buffer nread=%d ret_nread=%d\n",
				     nread, *ret_nread);
				goto leave;
			}
			a <<= 8;
			a |= *buffer++;
		}
		i = 0;
		val->d[j - 1] = a;
	}

leave:
	*ret_nread = nread;
	return val;
}
EXPORT_SYMBOL_GPL(mpi_read_from_buffer);

/****************
 * Make an mpi from a character string.
 */
int mpi_fromstr(MPI val, const char *str)
{
	int hexmode = 0, sign = 0, prepend_zero = 0, i, j, c, c1, c2;
	unsigned nbits, nbytes, nlimbs;
	mpi_limb_t a;

	if (*str == '-') {
		sign = 1;
		str++;
	}
	if (*str == '0' && str[1] == 'x')
		hexmode = 1;
	else
		return -EINVAL;	/* other bases are not yet supported */
	str += 2;

	nbits = strlen(str) * 4;
	if (nbits % 8)
		prepend_zero = 1;
	nbytes = (nbits + 7) / 8;
	nlimbs = (nbytes + BYTES_PER_MPI_LIMB - 1) / BYTES_PER_MPI_LIMB;
	if (val->alloced < nlimbs)
		if (!mpi_resize(val, nlimbs))
			return -ENOMEM;
	i = BYTES_PER_MPI_LIMB - nbytes % BYTES_PER_MPI_LIMB;
	i %= BYTES_PER_MPI_LIMB;
	j = val->nlimbs = nlimbs;
	val->sign = sign;
	for (; j > 0; j--) {
		a = 0;
		for (; i < BYTES_PER_MPI_LIMB; i++) {
			if (prepend_zero) {
				c1 = '0';
				prepend_zero = 0;
			} else
				c1 = *str++;
			assert(c1);
			c2 = *str++;
			assert(c2);
			if (c1 >= '0' && c1 <= '9')
				c = c1 - '0';
			else if (c1 >= 'a' && c1 <= 'f')
				c = c1 - 'a' + 10;
			else if (c1 >= 'A' && c1 <= 'F')
				c = c1 - 'A' + 10;
			else {
				mpi_clear(val);
				return 1;
			}
			c <<= 4;
			if (c2 >= '0' && c2 <= '9')
				c |= c2 - '0';
			else if (c2 >= 'a' && c2 <= 'f')
				c |= c2 - 'a' + 10;
			else if (c2 >= 'A' && c2 <= 'F')
				c |= c2 - 'A' + 10;
			else {
				mpi_clear(val);
				return 1;
			}
			a <<= 8;
			a |= c;
		}
		i = 0;

		val->d[j - 1] = a;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mpi_fromstr);

/****************
 * Special function to get the low 8 bytes from an mpi.
 * This can be used as a keyid; KEYID is an 2 element array.
 * Return the low 4 bytes.
 */
u32 mpi_get_keyid(const MPI a, u32 *keyid)
{
#if BYTES_PER_MPI_LIMB == 4
	if (keyid) {
		keyid[0] = a->nlimbs >= 2 ? a->d[1] : 0;
		keyid[1] = a->nlimbs >= 1 ? a->d[0] : 0;
	}
	return a->nlimbs >= 1 ? a->d[0] : 0;
#elif BYTES_PER_MPI_LIMB == 8
	if (keyid) {
		keyid[0] = a->nlimbs ? (u32) (a->d[0] >> 32) : 0;
		keyid[1] = a->nlimbs ? (u32) (a->d[0] & 0xffffffff) : 0;
	}
	return a->nlimbs ? (u32) (a->d[0] & 0xffffffff) : 0;
#else
#error Make this function work with other LIMB sizes
#endif
}

/****************
 * Return an allocated buffer with the MPI (msb first).
 * NBYTES receives the length of this buffer. Caller must free the
 * return string (This function does return a 0 byte buffer with NBYTES
 * set to zero if the value of A is zero. If sign is not NULL, it will
 * be set to the sign of the A.
 */
void *mpi_get_buffer(MPI a, unsigned *nbytes, int *sign)
{
	uint8_t *p, *buffer;
	mpi_limb_t alimb;
	int i;
	unsigned int n;

	if (sign)
		*sign = a->sign;
	*nbytes = n = a->nlimbs * BYTES_PER_MPI_LIMB;
	if (!n)
		n++;		/* avoid zero length allocation */
	p = buffer = kmalloc(n, GFP_KERNEL);

	for (i = a->nlimbs - 1; i >= 0; i--) {
		alimb = a->d[i];
#if BYTES_PER_MPI_LIMB == 4
		*p++ = alimb >> 24;
		*p++ = alimb >> 16;
		*p++ = alimb >> 8;
		*p++ = alimb;
#elif BYTES_PER_MPI_LIMB == 8
		*p++ = alimb >> 56;
		*p++ = alimb >> 48;
		*p++ = alimb >> 40;
		*p++ = alimb >> 32;
		*p++ = alimb >> 24;
		*p++ = alimb >> 16;
		*p++ = alimb >> 8;
		*p++ = alimb;
#else
#error please implement for this limb size.
#endif
	}

	/* this is sub-optimal but we need to do the shift operation
	 * because the caller has to free the returned buffer */
	for (p = buffer; !*p && *nbytes; p++, --*nbytes)
		;
	if (p != buffer)
		memmove(buffer, p, *nbytes);

	return buffer;
}
EXPORT_SYMBOL_GPL(mpi_get_buffer);

/****************
 * Use BUFFER to update MPI.
 */
int mpi_set_buffer(MPI a, const void *xbuffer, unsigned nbytes, int sign)
{
	const uint8_t *buffer = xbuffer, *p;
	mpi_limb_t alimb;
	int nlimbs;
	int i;

	nlimbs = (nbytes + BYTES_PER_MPI_LIMB - 1) / BYTES_PER_MPI_LIMB;
	if (RESIZE_IF_NEEDED(a, nlimbs) < 0)
		return -ENOMEM;
	a->sign = sign;

	for (i = 0, p = buffer + nbytes - 1; p >= buffer + BYTES_PER_MPI_LIMB;) {
#if BYTES_PER_MPI_LIMB == 4
		alimb = (mpi_limb_t) *p--;
		alimb |= (mpi_limb_t) *p-- << 8;
		alimb |= (mpi_limb_t) *p-- << 16;
		alimb |= (mpi_limb_t) *p-- << 24;
#elif BYTES_PER_MPI_LIMB == 8
		alimb = (mpi_limb_t) *p--;
		alimb |= (mpi_limb_t) *p-- << 8;
		alimb |= (mpi_limb_t) *p-- << 16;
		alimb |= (mpi_limb_t) *p-- << 24;
		alimb |= (mpi_limb_t) *p-- << 32;
		alimb |= (mpi_limb_t) *p-- << 40;
		alimb |= (mpi_limb_t) *p-- << 48;
		alimb |= (mpi_limb_t) *p-- << 56;
#else
#error please implement for this limb size.
#endif
		a->d[i++] = alimb;
	}
	if (p >= buffer) {
#if BYTES_PER_MPI_LIMB == 4
		alimb = *p--;
		if (p >= buffer)
			alimb |= (mpi_limb_t) *p-- << 8;
		if (p >= buffer)
			alimb |= (mpi_limb_t) *p-- << 16;
		if (p >= buffer)
			alimb |= (mpi_limb_t) *p-- << 24;
#elif BYTES_PER_MPI_LIMB == 8
		alimb = (mpi_limb_t) *p--;
		if (p >= buffer)
			alimb |= (mpi_limb_t) *p-- << 8;
		if (p >= buffer)
			alimb |= (mpi_limb_t) *p-- << 16;
		if (p >= buffer)
			alimb |= (mpi_limb_t) *p-- << 24;
		if (p >= buffer)
			alimb |= (mpi_limb_t) *p-- << 32;
		if (p >= buffer)
			alimb |= (mpi_limb_t) *p-- << 40;
		if (p >= buffer)
			alimb |= (mpi_limb_t) *p-- << 48;
		if (p >= buffer)
			alimb |= (mpi_limb_t) *p-- << 56;
#else
#error please implement for this limb size.
#endif
		a->d[i++] = alimb;
	}
	a->nlimbs = i;

	if (i != nlimbs) {
		pr_emerg("MPI: mpi_set_buffer: Assertion failed (%d != %d)", i,
		       nlimbs);
		BUG();
	}
	return 0;
}
EXPORT_SYMBOL_GPL(mpi_set_buffer);
