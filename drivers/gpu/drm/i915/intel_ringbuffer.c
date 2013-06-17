/*
 * Copyright © 2008-2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Eric Anholt <eric@anholt.net>
 *    Zou Nan hai <nanhai.zou@intel.com>
 *    Xiang Hai hao<haihao.xiang@intel.com>
 *
 */

#include "drmP.h"
#include "drm.h"
#include "i915_drv.h"
#include "i915_drm.h"
#include "i915_trace.h"
#include "intel_drv.h"

/*
 * 965+ support PIPE_CONTROL commands, which provide finer grained control
 * over cache flushing.
 */
struct pipe_control {
	struct drm_i915_gem_object *obj;
	volatile u32 *cpu_page;
	u32 gtt_offset;
};

static inline int ring_space(struct intel_ring_buffer *ring)
{
	int space = (ring->head & HEAD_ADDR) - (ring->tail + 8);
	if (space < 0)
		space += ring->size;
	return space;
}

static int
gen2_render_ring_flush(struct intel_ring_buffer *ring,
		       u32	invalidate_domains,
		       u32	flush_domains)
{
	u32 cmd;
	int ret;

	cmd = MI_FLUSH;
	if (((invalidate_domains|flush_domains) & I915_GEM_DOMAIN_RENDER) == 0)
		cmd |= MI_NO_WRITE_FLUSH;

	if (invalidate_domains & I915_GEM_DOMAIN_SAMPLER)
		cmd |= MI_READ_FLUSH;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring, cmd);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	return 0;
}

static int
gen4_render_ring_flush(struct intel_ring_buffer *ring,
		       u32	invalidate_domains,
		       u32	flush_domains)
{
	struct drm_device *dev = ring->dev;
	u32 cmd;
	int ret;

	/*
	* read/write caches:
	*
	* I915_GEM_DOMAIN_RENDER is always invalidated, but is
	* only flushed if MI_NO_WRITE_FLUSH is unset.  On 965, it is
	* also flushed at 2d versus 3d pipeline switches.
	*
	* read-only caches:
	*
	* I915_GEM_DOMAIN_SAMPLER is flushed on pre-965 if
	* MI_READ_FLUSH is set, and is always flushed on 965.
	*
	* I915_GEM_DOMAIN_COMMAND may not exist?
	*
	* I915_GEM_DOMAIN_INSTRUCTION, which exists on 965, is
	* invalidated when MI_EXE_FLUSH is set.
	*
	* I915_GEM_DOMAIN_VERTEX, which exists on 965, is
	* invalidated with every MI_FLUSH.
	*
	* TLBs:
	*
	* On 965, TLBs associated with I915_GEM_DOMAIN_COMMAND
	* and I915_GEM_DOMAIN_CPU in are invalidated at PTE write and
	* I915_GEM_DOMAIN_RENDER and I915_GEM_DOMAIN_SAMPLER
	* are flushed at any MI_FLUSH.
	*/

	cmd = MI_FLUSH | MI_NO_WRITE_FLUSH;
	if ((invalidate_domains|flush_domains) & I915_GEM_DOMAIN_RENDER)
		cmd &= ~MI_NO_WRITE_FLUSH;
	if (invalidate_domains & I915_GEM_DOMAIN_INSTRUCTION)
		cmd |= MI_EXE_FLUSH;

	if (invalidate_domains & I915_GEM_DOMAIN_COMMAND &&
	    (IS_G4X(dev) || IS_GEN5(dev)))
		cmd |= MI_INVALIDATE_ISP;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring, cmd);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	return 0;
}

/**
 * Emits a PIPE_CONTROL with a non-zero post-sync operation, for
 * implementing two workarounds on gen6.  From section 1.4.7.1
 * "PIPE_CONTROL" of the Sandy Bridge PRM volume 2 part 1:
 *
 * [DevSNB-C+{W/A}] Before any depth stall flush (including those
 * produced by non-pipelined state commands), software needs to first
 * send a PIPE_CONTROL with no bits set except Post-Sync Operation !=
 * 0.
 *
 * [Dev-SNB{W/A}]: Before a PIPE_CONTROL with Write Cache Flush Enable
 * =1, a PIPE_CONTROL with any non-zero post-sync-op is required.
 *
 * And the workaround for these two requires this workaround first:
 *
 * [Dev-SNB{W/A}]: Pipe-control with CS-stall bit set must be sent
 * BEFORE the pipe-control with a post-sync op and no write-cache
 * flushes.
 *
 * And this last workaround is tricky because of the requirements on
 * that bit.  From section 1.4.7.2.3 "Stall" of the Sandy Bridge PRM
 * volume 2 part 1:
 *
 *     "1 of the following must also be set:
 *      - Render Target Cache Flush Enable ([12] of DW1)
 *      - Depth Cache Flush Enable ([0] of DW1)
 *      - Stall at Pixel Scoreboard ([1] of DW1)
 *      - Depth Stall ([13] of DW1)
 *      - Post-Sync Operation ([13] of DW1)
 *      - Notify Enable ([8] of DW1)"
 *
 * The cache flushes require the workaround flush that triggered this
 * one, so we can't use it.  Depth stall would trigger the same.
 * Post-sync nonzero is what triggered this second workaround, so we
 * can't use that one either.  Notify enable is IRQs, which aren't
 * really our business.  That leaves only stall at scoreboard.
 */
static int
intel_emit_post_sync_nonzero_flush(struct intel_ring_buffer *ring)
{
	struct pipe_control *pc = ring->private;
	u32 scratch_addr = pc->gtt_offset + 128;
	int ret;


	ret = intel_ring_begin(ring, 6);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	intel_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			PIPE_CONTROL_STALL_AT_SCOREBOARD);
	intel_ring_emit(ring, scratch_addr | PIPE_CONTROL_GLOBAL_GTT); /* address */
	intel_ring_emit(ring, 0); /* low dword */
	intel_ring_emit(ring, 0); /* high dword */
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	ret = intel_ring_begin(ring, 6);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	intel_ring_emit(ring, PIPE_CONTROL_QW_WRITE);
	intel_ring_emit(ring, scratch_addr | PIPE_CONTROL_GLOBAL_GTT); /* address */
	intel_ring_emit(ring, 0);
	intel_ring_emit(ring, 0);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	return 0;
}

static int
gen6_render_ring_flush(struct intel_ring_buffer *ring,
                         u32 invalidate_domains, u32 flush_domains)
{
	u32 flags = 0;
	struct pipe_control *pc = ring->private;
	u32 scratch_addr = pc->gtt_offset + 128;
	int ret, i;

	/* Just flush everything.  Experiments have shown that reducing the
	 * number of bits based on the write domains has little performance
	 * impact.
	 */
	if (flush_domains) {
		flags |= PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH;
		flags |= PIPE_CONTROL_DEPTH_CACHE_FLUSH;
		/*
		 * Ensure that any following seqno writes only happen
		 * when the render cache is indeed flushed.
		 */
		flags |= PIPE_CONTROL_CS_STALL;
	}
	if (invalidate_domains) {
		flags |= PIPE_CONTROL_TLB_INVALIDATE;
		flags |= PIPE_CONTROL_INSTRUCTION_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_VF_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_CONST_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_STATE_CACHE_INVALIDATE;
		/*
		 * TLB invalidate requires a post-sync write.
		 */
		flags |= PIPE_CONTROL_QW_WRITE | PIPE_CONTROL_CS_STALL;
	}

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(4));
	intel_ring_emit(ring, flags);
	intel_ring_emit(ring, scratch_addr | PIPE_CONTROL_GLOBAL_GTT);
	intel_ring_emit(ring, 0);
	intel_ring_advance(ring);

	return 0;
}

static int
gen6_render_ring_flush__wa(struct intel_ring_buffer *ring,
			   u32 invalidate_domains, u32 flush_domains)
{
	int ret;

	/* Force SNB workarounds for PIPE_CONTROL flushes */
	ret = intel_emit_post_sync_nonzero_flush(ring);
	if (ret)
		return ret;

	return gen6_render_ring_flush(ring, invalidate_domains, flush_domains);
}

static void ring_write_tail(struct intel_ring_buffer *ring,
			    u32 value)
{
	drm_i915_private_t *dev_priv = ring->dev->dev_private;
	I915_WRITE_TAIL(ring, value);
}

u32 intel_ring_get_active_head(struct intel_ring_buffer *ring)
{
	drm_i915_private_t *dev_priv = ring->dev->dev_private;
	u32 acthd_reg = INTEL_INFO(ring->dev)->gen >= 4 ?
			RING_ACTHD(ring->mmio_base) : ACTHD;

	return I915_READ(acthd_reg);
}


void intel_ring_resample(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;

	if (!drm_core_check_feature(ring->dev, DRIVER_MODESET))
		i915_kernel_lost_context(ring->dev);
	else {
		ring->head = I915_READ_HEAD(ring);
		ring->tail = I915_READ_TAIL(ring) & TAIL_ADDR;
		ring->space = ring_space(ring);
		ring->last_retired_head = -1;
	}
}


static int init_ring_common(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj = ring->obj;
	int ret = 0;
	u32 head;

	/* TBD: Wake up relevant engine based on ring type */
	if (HAS_FORCE_WAKE(dev))
		gen6_gt_force_wake_get(dev_priv, FORCEWAKE_ALL);

	/* Stop the ring if it's running. */
	I915_WRITE_CTL(ring, 0);
	I915_WRITE_HEAD(ring, 0);
	ring->write_tail(ring, 0);

	head = I915_READ_HEAD(ring) & HEAD_ADDR;

	/* G45 ring initialization fails to reset head to zero */
	if (head != 0) {
		DRM_DEBUG_KMS("%s head not reset to zero "
			      "ctl %08x head %08x tail %08x start %08x\n",
			      ring->name,
			      I915_READ_CTL(ring),
			      I915_READ_HEAD(ring),
			      I915_READ_TAIL(ring),
			      I915_READ_START(ring));

		I915_WRITE_HEAD(ring, 0);

		if (I915_READ_HEAD(ring) & HEAD_ADDR) {
			DRM_ERROR("failed to set %s head to zero "
				  "ctl %08x head %08x tail %08x start %08x\n",
				  ring->name,
				  I915_READ_CTL(ring),
				  I915_READ_HEAD(ring),
				  I915_READ_TAIL(ring),
				  I915_READ_START(ring));
		}
	}

	/* Initialize the ring. This must happen _after_ we've cleared the ring
	 * registers with the above sequence (the readback of the HEAD registers
	 * also enforces ordering), otherwise the hw might lose the new ring
	 * register values. */
	I915_WRITE_START(ring, obj->gtt_offset);
	I915_WRITE_CTL(ring,
			((ring->size - PAGE_SIZE) & RING_NR_PAGES)
			| RING_VALID);

	/* If the head is still not zero, the ring is dead */
	if (wait_for((I915_READ_CTL(ring) & RING_VALID) != 0 &&
		     I915_READ_START(ring) == obj->gtt_offset &&
		     (I915_READ_HEAD(ring) & HEAD_ADDR) == 0, 50)) {
		DRM_ERROR("%s initialization failed "
				"ctl %08x head %08x tail %08x start %08x\n",
				ring->name,
				I915_READ_CTL(ring),
				I915_READ_HEAD(ring),
				I915_READ_TAIL(ring),
				I915_READ_START(ring));
		ret = -EIO;
		goto out;
	}

	if (!drm_core_check_feature(ring->dev, DRIVER_MODESET))
		i915_kernel_lost_context(ring->dev);
	else {
		ring->head = I915_READ_HEAD(ring);
		ring->tail = I915_READ_TAIL(ring) & TAIL_ADDR;
		ring->space = ring_space(ring);
		ring->last_retired_head = -1;
	}

out:
	/* TBD: Wake up relevant engine based on ring type */
	if (HAS_FORCE_WAKE(dev))
		gen6_gt_force_wake_put(dev_priv, FORCEWAKE_ALL);

	return ret;
}

static int
init_pipe_control(struct intel_ring_buffer *ring)
{
	struct pipe_control *pc;
	struct drm_i915_gem_object *obj;
	int ret;

	if (ring->private)
		return 0;

	pc = kmalloc(sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	obj = i915_gem_alloc_object(ring->dev, 4096);
	if (obj == NULL) {
		DRM_ERROR("Failed to allocate seqno page\n");
		ret = -ENOMEM;
		goto err;
	}

	i915_gem_object_set_cache_level(obj, I915_CACHE_LLC);

	ret = i915_gem_object_pin(obj, 4096, true);
	if (ret)
		goto err_unref;

	pc->gtt_offset = obj->gtt_offset;
	pc->cpu_page =  kmap(obj->pages[0]);
	if (pc->cpu_page == NULL)
		goto err_unpin;

	pc->obj = obj;
	ring->private = pc;
	return 0;

err_unpin:
	i915_gem_object_unpin(obj);
err_unref:
	drm_gem_object_unreference(&obj->base);
err:
	kfree(pc);
	return ret;
}

static void
cleanup_pipe_control(struct intel_ring_buffer *ring)
{
	struct pipe_control *pc = ring->private;
	struct drm_i915_gem_object *obj;

	if (!ring->private)
		return;

	obj = pc->obj;
	kunmap(obj->pages[0]);
	i915_gem_object_unpin(obj);
	drm_gem_object_unreference(&obj->base);

	kfree(pc);
	ring->private = NULL;
}

u32
get_pipe_control_scratch_addr(struct intel_ring_buffer *ring)
{
	struct pipe_control *pc = ring->private;
	if (!ring->private)
		return 0;
	return pc->gtt_offset;
}

static int init_render_ring(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret = init_ring_common(ring);
	uint32_t imr;

	if (INTEL_INFO(dev)->gen > 3) {
		if (!IS_VALLEYVIEW(dev))
			I915_WRITE(MI_MODE,
				_MASKED_BIT_ENABLE(VS_TIMER_DISPATCH));
		if (IS_GEN7(dev)) {
			if (IS_VALLEYVIEW(dev)) {
				I915_WRITE(GFX_MODE_GEN7,
					   _MASKED_BIT_ENABLE(GFX_REPLAY_MODE));
				I915_WRITE(MI_MODE, I915_READ(MI_MODE) |
					   _MASKED_BIT_ENABLE(MI_FLUSH_ENABLE));
			} else
				I915_WRITE(GFX_MODE_GEN7,
				_MASKED_BIT_DISABLE(GFX_TLB_INVALIDATE_ALWAYS) |
				_MASKED_BIT_ENABLE(GFX_REPLAY_MODE));
		}
	}

	if (INTEL_INFO(dev)->gen >= 5) {
		ret = init_pipe_control(ring);
		if (ret)
			return ret;
	}

	if (IS_GEN6(dev)) {
		/* From the Sandybridge PRM, volume 1 part 3, page 24:
		 * "If this bit is set, STCunit will have LRA as replacement
		 *  policy. [...] This bit must be reset.  LRA replacement
		 *  policy is not supported."
		 */
		I915_WRITE(CACHE_MODE_0_OFFSET(dev),
			   _MASKED_BIT_DISABLE(CM0_STC_EVICT_DISABLE_LRA_SNB));

		/* This is not explicitly set for GEN6, so read the register.
		 * see intel_ring_mi_set_context() for why we care.
		 * TODO: consider explicitly setting the bit for GEN5
		 */
		ring->itlb_before_ctx_switch =
			!!(I915_READ(GFX_MODE) & GFX_TLB_INVALIDATE_ALWAYS);
	}

	if ((INTEL_INFO(dev)->gen >= 6) && !(IS_VALLEYVIEW(dev)))
		I915_WRITE(INSTPM, _MASKED_BIT_ENABLE(INSTPM_FORCE_ORDERING));

	imr = ~0;
	if (INTEL_INFO(dev)->gen >= 7)
		imr &= ~GEN6_RENDER_TIMEOUT_COUNTER_EXPIRED;

	if (HAS_L3_GPU_CACHE(dev))
		imr &= ~GEN6_RENDER_L3_PARITY_ERROR;
	if (IS_VALLEYVIEW(dev))
		imr &= ~GT_GEN6_PERFMON_BUFFER_INTERRUPT;

	I915_WRITE_IMR(ring, imr);

	return ret;
}

static void render_ring_cleanup(struct intel_ring_buffer *ring)
{
	if (!ring->private)
		return;

	cleanup_pipe_control(ring);
}

static void
update_mboxes(struct intel_ring_buffer *ring,
	    u32 seqno,
	    u32 mmio_offset)
{
	intel_ring_emit(ring, MI_SEMAPHORE_MBOX |
			      MI_SEMAPHORE_GLOBAL_GTT |
			      MI_SEMAPHORE_REGISTER |
			      MI_SEMAPHORE_UPDATE);
	intel_ring_emit(ring, seqno);
	intel_ring_emit(ring, mmio_offset);
}

/**
 * gen6_add_request - Update the semaphore mailbox registers
 * 
 * @ring - ring that is adding a request
 * @seqno - return seqno stuck into the ring
 *
 * Update the mailbox registers in the *other* rings with the current seqno.
 * This acts like a signal in the canonical semaphore.
 */
static int
gen6_add_request(struct intel_ring_buffer *ring,
		 u32 *seqno)
{
	u32 mbox1_reg;
	u32 mbox2_reg;
	int ret;

	ret = intel_ring_begin(ring, 10);
	if (ret)
		return ret;

	mbox1_reg = ring->signal_mbox[0];
	mbox2_reg = ring->signal_mbox[1];

	*seqno = i915_gem_next_request_seqno(ring);

	update_mboxes(ring, *seqno, mbox1_reg);
	update_mboxes(ring, *seqno, mbox2_reg);
	intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
	intel_ring_emit(ring, I915_GEM_HWS_INDEX << MI_STORE_DWORD_INDEX_SHIFT);
	intel_ring_emit(ring, *seqno);
	intel_ring_emit(ring, MI_USER_INTERRUPT);
	intel_ring_advance(ring);

	return 0;
}

/**
 * intel_ring_sync - sync the waiter to the signaller on seqno
 *
 * @waiter - ring that is waiting
 * @signaller - ring which has, or will signal
 * @seqno - seqno which the waiter will block on
 */
static int
gen6_ring_sync(struct intel_ring_buffer *waiter,
	       struct intel_ring_buffer *signaller,
	       u32 seqno)
{
	int ret;
	u32 dw1 = MI_SEMAPHORE_MBOX |
		  MI_SEMAPHORE_COMPARE |
		  MI_SEMAPHORE_REGISTER;

	/* Throughout all of the GEM code, seqno passed implies our current
	 * seqno is >= the last seqno executed. However for hardware the
	 * comparison is strictly greater than.
	 */
	seqno -= 1;

	WARN_ON(signaller->semaphore_register[waiter->id] ==
		MI_SEMAPHORE_SYNC_INVALID);

	ret = intel_ring_begin(waiter, 4);
	if (ret)
		return ret;

	intel_ring_emit(waiter,
			dw1 | signaller->semaphore_register[waiter->id]);
	intel_ring_emit(waiter, seqno);
	intel_ring_emit(waiter, 0);
	intel_ring_emit(waiter, MI_NOOP);
	intel_ring_advance(waiter);

	return 0;
}

#define PIPE_CONTROL_FLUSH(ring__, addr__)					\
do {									\
	intel_ring_emit(ring__, GFX_OP_PIPE_CONTROL(4) | PIPE_CONTROL_QW_WRITE |		\
		 PIPE_CONTROL_DEPTH_STALL);				\
	intel_ring_emit(ring__, (addr__) | PIPE_CONTROL_GLOBAL_GTT);			\
	intel_ring_emit(ring__, 0);							\
	intel_ring_emit(ring__, 0);							\
} while (0)

static int
pc_render_add_request(struct intel_ring_buffer *ring,
		      u32 *result)
{
	u32 seqno = i915_gem_next_request_seqno(ring);
	struct pipe_control *pc = ring->private;
	u32 scratch_addr = pc->gtt_offset + 128;
	int ret;

	/* For Ironlake, MI_USER_INTERRUPT was deprecated and apparently
	 * incoherent with writes to memory, i.e. completely fubar,
	 * so we need to use PIPE_NOTIFY instead.
	 *
	 * However, we also need to workaround the qword write
	 * incoherence by flushing the 6 PIPE_NOTIFY buffers out to
	 * memory before requesting an interrupt.
	 */
	ret = intel_ring_begin(ring, 32);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(4) | PIPE_CONTROL_QW_WRITE |
			PIPE_CONTROL_WRITE_FLUSH |
			PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE);
	intel_ring_emit(ring, pc->gtt_offset | PIPE_CONTROL_GLOBAL_GTT);
	intel_ring_emit(ring, seqno);
	intel_ring_emit(ring, 0);
	PIPE_CONTROL_FLUSH(ring, scratch_addr);
	scratch_addr += 128; /* write to separate cachelines */
	PIPE_CONTROL_FLUSH(ring, scratch_addr);
	scratch_addr += 128;
	PIPE_CONTROL_FLUSH(ring, scratch_addr);
	scratch_addr += 128;
	PIPE_CONTROL_FLUSH(ring, scratch_addr);
	scratch_addr += 128;
	PIPE_CONTROL_FLUSH(ring, scratch_addr);
	scratch_addr += 128;
	PIPE_CONTROL_FLUSH(ring, scratch_addr);

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(4) | PIPE_CONTROL_QW_WRITE |
			PIPE_CONTROL_WRITE_FLUSH |
			PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE |
			PIPE_CONTROL_NOTIFY);
	intel_ring_emit(ring, pc->gtt_offset | PIPE_CONTROL_GLOBAL_GTT);
	intel_ring_emit(ring, seqno);
	intel_ring_emit(ring, 0);
	intel_ring_advance(ring);

	*result = seqno;
	return 0;
}

static u32
gen6_ring_get_seqno(struct intel_ring_buffer *ring, bool lazy_coherency)
{
	/* Workaround to force correct ordering between irq and seqno writes on
	 * ivb (and maybe also on snb) by reading from a CS register (like
	 * ACTHD) before reading the status page. */
	if (!lazy_coherency)
		intel_ring_get_active_head(ring);
	return intel_read_status_page(ring, I915_GEM_HWS_INDEX);
}

static u32
ring_get_seqno(struct intel_ring_buffer *ring, bool lazy_coherency)
{
	return intel_read_status_page(ring, I915_GEM_HWS_INDEX);
}

static u32
pc_render_get_seqno(struct intel_ring_buffer *ring, bool lazy_coherency)
{
	struct pipe_control *pc = ring->private;
	return pc->cpu_page[0];
}

static bool
gen5_ring_get_irq(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long flags;

	if (!dev->irq_enabled)
		return false;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (ring->irq_refcount++ == 0) {
		dev_priv->gt_irq_mask &= ~ring->irq_enable_mask;
		I915_WRITE(GTIMR, dev_priv->gt_irq_mask);
		POSTING_READ(GTIMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void
gen5_ring_put_irq(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--ring->irq_refcount == 0) {
		dev_priv->gt_irq_mask |= ring->irq_enable_mask;
		I915_WRITE(GTIMR, dev_priv->gt_irq_mask);
		POSTING_READ(GTIMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

static bool
i9xx_ring_get_irq(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long flags;

	if (!dev->irq_enabled)
		return false;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (ring->irq_refcount++ == 0) {
		dev_priv->irq_mask &= ~ring->irq_enable_mask;
		I915_WRITE(IMR, dev_priv->irq_mask);
		POSTING_READ(IMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void
i9xx_ring_put_irq(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--ring->irq_refcount == 0) {
		dev_priv->irq_mask |= ring->irq_enable_mask;
		I915_WRITE(IMR, dev_priv->irq_mask);
		POSTING_READ(IMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

static bool
i8xx_ring_get_irq(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long flags;

	if (!dev->irq_enabled)
		return false;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (ring->irq_refcount++ == 0) {
		dev_priv->irq_mask &= ~ring->irq_enable_mask;
		I915_WRITE16(IMR, dev_priv->irq_mask);
		POSTING_READ16(IMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void
i8xx_ring_put_irq(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--ring->irq_refcount == 0) {
		dev_priv->irq_mask |= ring->irq_enable_mask;
		I915_WRITE16(IMR, dev_priv->irq_mask);
		POSTING_READ16(IMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

void intel_ring_setup_status_page(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = ring->dev->dev_private;
	u32 mmio = 0;

	/* The ring status page addresses are no longer next to the rest of
	 * the ring registers as of gen7.
	 */
	if (IS_GEN7(dev)) {
		switch (ring->id) {
		case RCS:
			mmio = RENDER_HWS_PGA_GEN7;
			break;
		case BCS:
			mmio = BLT_HWS_PGA_GEN7;
			break;
		case VCS:
			mmio = BSD_HWS_PGA_GEN7;
			break;
		}
	} else if (IS_GEN6(ring->dev)) {
		mmio = RING_HWS_PGA_GEN6(ring->mmio_base);
	} else {
		mmio = RING_HWS_PGA(ring->mmio_base);
	}

	I915_WRITE(mmio, (u32)ring->status_page.gfx_addr);
	POSTING_READ(mmio);
}

static int
bsd_ring_flush(struct intel_ring_buffer *ring,
	       u32     invalidate_domains,
	       u32     flush_domains)
{
	int ret;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring, MI_FLUSH);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);
	return 0;
}

static int
i9xx_add_request(struct intel_ring_buffer *ring,
		 u32 *result)
{
	u32 seqno;
	int ret;

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	seqno = i915_gem_next_request_seqno(ring);

	intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
	intel_ring_emit(ring, I915_GEM_HWS_INDEX << MI_STORE_DWORD_INDEX_SHIFT);
	intel_ring_emit(ring, seqno);
	intel_ring_emit(ring, MI_USER_INTERRUPT);
	intel_ring_advance(ring);

	*result = seqno;
	return 0;
}

static bool
gen6_ring_get_irq(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long flags;
	uint32_t imr;

	if (!dev->irq_enabled)
	       return false;

	/* It looks like we need to prevent the gt from suspending while waiting
	 * for an notify irq, otherwise irqs seem to get lost on at least the
	 * blt/bsd rings on ivb. */

	/* TBD: Wake up relevant engine based on ring type */
	gen6_gt_force_wake_get(dev_priv, FORCEWAKE_ALL);

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (ring->irq_refcount++ == 0) {
		imr = I915_READ_IMR(ring);
		imr &= ~ring->irq_enable_mask;
		I915_WRITE_IMR(ring, imr);

		dev_priv->gt_irq_mask &= ~ring->irq_enable_mask;
		I915_WRITE(GTIMR, dev_priv->gt_irq_mask);
		POSTING_READ(GTIMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void
gen6_ring_put_irq(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long flags;
	uint32_t imr;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--ring->irq_refcount == 0) {
		imr = I915_READ_IMR(ring);
		imr |= ring->irq_enable_mask;
		I915_WRITE_IMR(ring, imr);

		dev_priv->gt_irq_mask |= ring->irq_enable_mask;
		I915_WRITE(GTIMR, dev_priv->gt_irq_mask);
		POSTING_READ(GTIMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	/* TBD: Wake up relevant engine based on ring type */
	gen6_gt_force_wake_put(dev_priv, FORCEWAKE_ALL);
}

static int
i965_dispatch_execbuffer(struct intel_ring_buffer *ring,
			u32 offset, u32 len,
			void *priv_data, u32 priv_length)
{
	int ret;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring,
			MI_BATCH_BUFFER_START |
			MI_BATCH_GTT |
			MI_BATCH_NON_SECURE_I965);
	intel_ring_emit(ring, offset);
	intel_ring_advance(ring);

	return 0;
}

static int
i830_dispatch_execbuffer(struct intel_ring_buffer *ring,
				u32 offset, u32 len,
				void *priv_data, u32 priv_length)
{
	int ret;

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	intel_ring_emit(ring, MI_BATCH_BUFFER);
	intel_ring_emit(ring, offset | MI_BATCH_NON_SECURE);
	intel_ring_emit(ring, offset + len - 8);
	intel_ring_emit(ring, 0);
	intel_ring_advance(ring);

	return 0;
}

static int
i915_dispatch_execbuffer(struct intel_ring_buffer *ring,
				u32 offset, u32 len,
				void *priv_data, u32 priv_length)
{
	int ret;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring, MI_BATCH_BUFFER_START | MI_BATCH_GTT);
	intel_ring_emit(ring, offset | MI_BATCH_NON_SECURE);
	intel_ring_advance(ring);

	return 0;
}

static void cleanup_status_page(struct intel_ring_buffer *ring)
{
	struct drm_i915_gem_object *obj;

	obj = ring->status_page.obj;
	if (obj == NULL)
		return;

	kunmap(obj->pages[0]);
	i915_gem_object_unpin(obj);
	drm_gem_object_unreference(&obj->base);
	ring->status_page.obj = NULL;
}

static int init_status_page(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_gem_object *obj;
	int ret;

	obj = i915_gem_alloc_object(dev, 4096);
	if (obj == NULL) {
		DRM_ERROR("Failed to allocate status page\n");
		ret = -ENOMEM;
		goto err;
	}

	i915_gem_object_set_cache_level(obj, I915_CACHE_LLC);

	ret = i915_gem_object_pin(obj, 4096, true);
	if (ret != 0) {
		goto err_unref;
	}

	ring->status_page.gfx_addr = obj->gtt_offset;
	ring->status_page.page_addr = kmap(obj->pages[0]);
	if (ring->status_page.page_addr == NULL) {
		ret = -ENOMEM;
		goto err_unpin;
	}
	ring->status_page.obj = obj;
	memset(ring->status_page.page_addr, 0, PAGE_SIZE);

	intel_ring_setup_status_page(ring);
	DRM_DEBUG_DRIVER("%s hws offset: 0x%08x\n",
			ring->name, ring->status_page.gfx_addr);

	return 0;

err_unpin:
	i915_gem_object_unpin(obj);
err_unref:
	drm_gem_object_unreference(&obj->base);
err:
	return ret;
}

static int intel_init_ring_buffer(struct drm_device *dev,
				  struct intel_ring_buffer *ring)
{
	struct drm_i915_gem_object *obj;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	ring->dev = dev;
	INIT_LIST_HEAD(&ring->active_list);
	INIT_LIST_HEAD(&ring->request_list);
	ring->size = 32 * PAGE_SIZE;

	init_waitqueue_head(&ring->irq_queue);

	if (I915_NEED_GFX_HWS(dev)) {
		ret = init_status_page(ring);
		if (ret)
			return ret;
	}

	obj = i915_gem_alloc_object(dev, ring->size);
	if (obj == NULL) {
		DRM_ERROR("Failed to allocate ringbuffer\n");
		ret = -ENOMEM;
		goto err_hws;
	}

	/* mark ring buffers as read-only from GPU side by default */
	obj->gt_ro = 1;

	ring->obj = obj;

	ret = i915_gem_object_pin(obj, PAGE_SIZE, true);
	if (ret)
		goto err_unref;

	ret = i915_gem_object_set_to_gtt_domain(obj, true);
	if (ret)
		goto err_unpin;

	ring->virtual_start =
		ioremap_wc(dev_priv->mm.gtt->gma_bus_addr + obj->gtt_offset,
			   ring->size);
	if (ring->virtual_start == NULL) {
		DRM_ERROR("Failed to map ringbuffer.\n");
		ret = -EINVAL;
		goto err_unpin;
	}

	ret = ring->init(ring);
	if (ret)
		goto err_unmap;

	/* Workaround an erratum on the i830 which causes a hang if
	 * the TAIL pointer points to within the last 2 cachelines
	 * of the buffer.
	 */
	ring->effective_size = ring->size;
	if (IS_I830(ring->dev) || IS_845G(ring->dev))
		ring->effective_size -= 128;

	return 0;

err_unmap:
	iounmap(ring->virtual_start);
err_unpin:
	i915_gem_object_unpin(obj);
err_unref:
	drm_gem_object_unreference(&obj->base);
	ring->obj = NULL;
err_hws:
	cleanup_status_page(ring);
	return ret;
}

void intel_cleanup_ring_buffer(struct intel_ring_buffer *ring)
{
	struct drm_i915_private *dev_priv;
	int ret;

	if (ring->obj == NULL)
		return;

	/* Disable the ring buffer. The ring must be idle at this point */
	dev_priv = ring->dev->dev_private;
	ret = intel_wait_ring_idle(ring);
	if (ret)
		DRM_ERROR("failed to quiesce %s whilst cleaning up: %d\n",
			  ring->name, ret);

	I915_WRITE_CTL(ring, 0);

	iounmap(ring->virtual_start);

	i915_gem_object_unpin(ring->obj);
	drm_gem_object_unreference(&ring->obj->base);
	ring->obj = NULL;

	if (ring->cleanup)
		ring->cleanup(ring);

	cleanup_status_page(ring);
}

static int intel_wrap_ring_buffer(struct intel_ring_buffer *ring)
{
	uint32_t __iomem *virt;
	int rem = ring->size - ring->tail;

	if (ring->space < rem) {
		int ret = intel_wait_ring_buffer(ring, rem);
		if (ret)
			return ret;
	}

	virt = ring->virtual_start + ring->tail;
	rem /= 4;
	while (rem--)
		iowrite32(MI_NOOP, virt++);

	ring->tail = 0;
	ring->space = ring_space(ring);

	return 0;
}

static int intel_ring_wait_seqno(struct intel_ring_buffer *ring, u32 seqno)
{
	int ret;

	ret = i915_wait_seqno(ring, seqno);
	if (!ret)
		i915_gem_retire_requests_ring(ring);

	return ret;
}

static int intel_ring_wait_request(struct intel_ring_buffer *ring, int n)
{
	struct drm_i915_gem_request *request;
	u32 seqno = 0;
	int ret;

	i915_gem_retire_requests_ring(ring);

	if (ring->last_retired_head != -1) {
		ring->head = ring->last_retired_head;
		ring->last_retired_head = -1;
		ring->space = ring_space(ring);
		if (ring->space >= n)
			return 0;
	}

	list_for_each_entry(request, &ring->request_list, list) {
		int space;

		if (request->tail == -1)
			continue;

		space = request->tail - (ring->tail + 8);
		if (space < 0)
			space += ring->size;
		if (space >= n) {
			seqno = request->seqno;
			break;
		}

		/* Consume this request in case we need more space than
		 * is available and so need to prevent a race between
		 * updating last_retired_head and direct reads of
		 * I915_RING_HEAD. It also provides a nice sanity check.
		 */
		request->tail = -1;
	}

	if (seqno == 0)
		return -ENOSPC;

	ret = intel_ring_wait_seqno(ring, seqno);
	if (ret)
		return ret;

	if (WARN_ON(ring->last_retired_head == -1))
		return -ENOSPC;

	ring->head = ring->last_retired_head;
	ring->last_retired_head = -1;
	ring->space = ring_space(ring);
	if (WARN_ON(ring->space < n))
		return -ENOSPC;

	return 0;
}

int intel_wait_ring_buffer(struct intel_ring_buffer *ring, int n)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long end;
	int ret;

	ret = intel_ring_wait_request(ring, n);
	if (ret != -ENOSPC)
		return ret;

	trace_i915_ring_wait_begin(ring);
	/* With GEM the hangcheck timer should kick us out of the loop,
	 * leaving it early runs the risk of corrupting GEM state (due
	 * to running on almost untested codepaths). But on resume
	 * timers don't work yet, so prevent a complete hang in that
	 * case by choosing an insanely large timeout. */
	end = jiffies + 60 * HZ;

	do {
		ring->head = I915_READ_HEAD(ring);
		ring->space = ring_space(ring);
		if (ring->space >= n) {
			trace_i915_ring_wait_end(ring);
			return 0;
		}

		if (dev->primary->master) {
			struct drm_i915_master_private *master_priv = dev->primary->master->driver_priv;
			if (master_priv->sarea_priv)
				master_priv->sarea_priv->perf_boxes |= I915_BOX_WAIT;
		}

		msleep(1);

		ret = i915_gem_check_wedge(dev_priv,
				dev_priv->mm.interruptible, ring);

		if (ret)
			return ret;
	} while (!time_after(jiffies, end));
	trace_i915_ring_wait_end(ring);
	return -EBUSY;
}

int intel_ring_begin(struct intel_ring_buffer *ring,
		     int num_dwords)
{
	drm_i915_private_t *dev_priv = ring->dev->dev_private;
	int n = 4*num_dwords;
	int ret;

	ret = i915_gem_check_wedge(dev_priv, dev_priv->mm.interruptible, ring);
	if (ret)
		return ret;

	if (unlikely(ring->tail + n > ring->effective_size)) {
		ret = intel_wrap_ring_buffer(ring);
		if (unlikely(ret))
			return ret;
	}

	if (unlikely(ring->space < n)) {
		ret = intel_wait_ring_buffer(ring, n);
		if (unlikely(ret))
			return ret;
	}

	ring->space -= n;
	return 0;
}

void intel_ring_advance(struct intel_ring_buffer *ring)
{
	struct drm_i915_private *dev_priv = ring->dev->dev_private;

	ring->tail &= ring->size - 1;

	/* Re-schedule the hangcheck timer each time the ring is given new work
	* so that we can detect hangs caused by commands inserted directly
	* to the ring as well as bad batch buffers */
	if (!dev_priv->mm.suspended && i915_enable_hangcheck) {
		mod_timer(&dev_priv->hangcheck[ring->id].timer,
			jiffies + DRM_I915_HANGCHECK_JIFFIES);
	}

	if (dev_priv->stop_rings & intel_ring_flag(ring))
		return;

	ring->write_tail(ring, ring->tail);
}


static void gen6_bsd_ring_write_tail(struct intel_ring_buffer *ring,
				     u32 value)
{
	drm_i915_private_t *dev_priv = ring->dev->dev_private;

       /* Every tail move must follow the sequence below */

	/* Disable notification that the ring is IDLE. The GT
	 * will then assume that it is busy and bring it out of rc6.
	 */
	I915_WRITE(GEN6_BSD_SLEEP_PSMI_CONTROL,
		   _MASKED_BIT_ENABLE(GEN6_BSD_SLEEP_MSG_DISABLE));

	/* Clear the context id. Here be magic! */
	I915_WRITE64(GEN6_BSD_RNCID, 0x0);

	/* Wait for the ring not to be idle, i.e. for it to wake up. */
	if (wait_for((I915_READ(GEN6_BSD_SLEEP_PSMI_CONTROL) &
		      GEN6_BSD_SLEEP_INDICATOR) == 0,
		     50))
		DRM_ERROR("timed out waiting for the BSD ring to wake up\n");

	/* Now that the ring is fully powered up, update the tail */
	I915_WRITE_TAIL(ring, value);
	POSTING_READ(RING_TAIL(ring->mmio_base));

	/* Let the ring send IDLE messages to the GT again,
	 * and so let it sleep to conserve power when idle.
	 */
	I915_WRITE(GEN6_BSD_SLEEP_PSMI_CONTROL,
		   _MASKED_BIT_DISABLE(GEN6_BSD_SLEEP_MSG_DISABLE));
}

static int gen6_ring_flush(struct intel_ring_buffer *ring,
			   u32 invalidate, u32 flush)
{
	uint32_t cmd;
	int ret;

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	cmd = MI_FLUSH_DW;
	/*
	 * Bspec vol 1c.5 - video engine command streamer:
	 * "If ENABLED, all TLBs will be invalidated once the flush
	 * operation is complete. This bit is only valid when the
	 * Post-Sync Operation field is a value of 1h or 3h."
	 */
	if (invalidate & I915_GEM_GPU_DOMAINS)
		cmd |= MI_INVALIDATE_TLB | MI_INVALIDATE_BSD |
			MI_FLUSH_DW_STORE_INDEX | MI_FLUSH_DW_OP_STOREDW;
	intel_ring_emit(ring, cmd);
	intel_ring_emit(ring, I915_GEM_SCRATCH_INDEX << 3);
	intel_ring_emit(ring, 0);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);
	return 0;
}

static int
vlv_launch_cb2(struct intel_ring_buffer *ring)
{
	int			i;
	int			ret = 0;
	uint32_t		hws_pga;
	drm_i915_private_t	*dev_priv = ring->dev->dev_private;

	/* Get HW Status Page address & point to its center */
	hws_pga = 0x800 + (I915_READ(HWS_PGA) & 0xFFFFF000);

	/* Insert 20 Store Data Immediate commands */
	for (i = 0; i < 20; i++) {
		ret = intel_ring_begin(ring, 4);
		if (ret)
			return ret;

		intel_ring_emit(ring, 0x10400002); /* SDI - DW0 */
		intel_ring_emit(ring, 0);	/* SDI - DW1 */
		intel_ring_emit(ring, hws_pga);	/* SDI - Address */
		intel_ring_emit(ring, 0);	/* SDI - Data */
		intel_ring_advance(ring);
	}

	ret = intel_ring_begin(ring, 20);
	if (ret)
		return ret;

	/* Pipe Control */
	intel_ring_emit(ring, 0x7a000003);	/* PipeControl DW0 */
	intel_ring_emit(ring, 0x01010a0);	/* DW1 */
	intel_ring_emit(ring, 0);		/* DW2 */
	intel_ring_emit(ring, 0);		/* DW3 */
	intel_ring_emit(ring, 0);		/* DW4 */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */

	/* Start CB2 */
	intel_ring_emit(ring, 0x18800800);	/* BB Start - CB2 */
	intel_ring_emit(ring, 0);		/* Address */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */

	/* Pipe Control */
	intel_ring_emit(ring, 0x7a000003);	/* PipeControl DW0 */
	intel_ring_emit(ring, 0x01010a0);	/* DW1 */
	intel_ring_emit(ring, 0);		/* DW2 */
	intel_ring_emit(ring, 0);		/* DW3 */
	intel_ring_emit(ring, 0);		/* DW4 */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */

	intel_ring_advance(ring);

	/* Add another 20 Store Data Immediate commands */
	for (i = 0; i < 20; i++) {
		ret = intel_ring_begin(ring, 4);
		if (ret)
			return ret;

		intel_ring_emit(ring, 0x10400002); /* SDI - DW0 */
		intel_ring_emit(ring, 0);	/* SDI - DW1 */
		intel_ring_emit(ring, hws_pga);	/* SDI - Address */
		intel_ring_emit(ring, 0);	/* SDI - Data */
		intel_ring_advance(ring);
	}

	return ret;
}

static int
gen6_ring_dispatch_execbuffer(struct intel_ring_buffer *ring,
				u32 offset, u32 len,
				void *priv_data, u32 priv_length)
{
	int ret = 0;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring, MI_BATCH_BUFFER_START | MI_BATCH_NON_SECURE_I965);
	/* bit0-7 is the length on GEN6+ */
	intel_ring_emit(ring, offset);
	intel_ring_advance(ring);

	/* Execute CB2 if requested to do so */
	if ((priv_length == sizeof(u32)) &&
	    (*(u32 *)priv_data == 0xffffffff)) {
		if (IS_VALLEYVIEW(ring->dev))
			ret = vlv_launch_cb2(ring);
	}

	return ret;
}

/* Blitter support (SandyBridge+) */

static int blt_ring_flush(struct intel_ring_buffer *ring,
			  u32 invalidate, u32 flush)
{
	uint32_t cmd;
	int ret;

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	cmd = MI_FLUSH_DW;
	/*
	 * Bspec vol 1c.3 - blitter engine command streamer:
	 * "If ENABLED, all TLBs will be invalidated once the flush
	 * operation is complete. This bit is only valid when the
	 * Post-Sync Operation field is a value of 1h or 3h."
	 */
	if (invalidate & I915_GEM_DOMAIN_RENDER)
		cmd |= MI_INVALIDATE_TLB | MI_FLUSH_DW_STORE_INDEX |
			MI_FLUSH_DW_OP_STOREDW;
	intel_ring_emit(ring, cmd);
	intel_ring_emit(ring, I915_GEM_SCRATCH_INDEX << 3);
	intel_ring_emit(ring, 0);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);
	return 0;
}


int intel_ring_disable(struct intel_ring_buffer *ring)
{
	if (ring && ring->disable)
		return ring->disable(ring);
	else {
		DRM_ERROR("ring disable not supported\n");
		return -EINVAL;
	}
}


static int
gen6_ring_disable(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	uint32_t ring_ctl;
	uint32_t mi_mode;
	uint32_t retries = 10000;

	/* Request the ring to go idle */
	I915_WRITE_MODE(ring, _MASKED_BIT_ENABLE(MODE_STOP));

	/* Wait for idle */
	do {
		mi_mode = I915_READ_MODE(ring);
	} while (retries-- && !(mi_mode & MODE_IDLE));

	if (retries == 0) {
		DRM_ERROR("timed out trying to disable ring %d\n", ring->id);
		return -ETIMEDOUT;
	}

	/* Disable the ring */
	ring_ctl = I915_READ_CTL(ring);
	ring_ctl &= (RING_NR_PAGES | RING_REPORT_MASK);
	I915_WRITE_CTL(ring, ring_ctl);
	ring_ctl = I915_READ_CTL(ring);  /* Barrier read */

	return ((ring_ctl & RING_VALID) == 0) ? 0 : -EIO;
}


int intel_ring_enable(struct intel_ring_buffer *ring)
{
	if (ring && ring->enable)
		return ring->enable(ring);
	else {
		DRM_ERROR("ring enable not supported\n");
		return -EINVAL;
	}
}


static int
gen6_ring_enable(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	uint32_t ring_ctl;
	uint32_t mode;

	/* Clear the MI_MODE stop bit */
	I915_WRITE_MODE(ring, _MASKED_BIT_DISABLE(MODE_STOP));
	mode = I915_READ_MODE(ring);    /* Barrier read */

	/* Enable the ring */
	ring_ctl = I915_READ_CTL(ring);
	ring_ctl &= (RING_NR_PAGES | RING_REPORT_MASK);
	I915_WRITE_CTL(ring, ring_ctl | RING_VALID);
	ring_ctl = I915_READ_CTL(ring); /* Barrier read */

	return ((ring_ctl & RING_VALID) == 0) ? -EIO : 0;
}


int intel_ring_reset(struct intel_ring_buffer *ring)
{
	if (ring && ring->reset)
		return ring->reset(ring);
	else {
		DRM_ERROR("ring reset not supported\n");
		return -EINVAL;
	}
}

static int
gen6_ring_reset(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret = 0;
	char *reset_event[2];
	unsigned long irqflags;
	reset_event[1] = NULL;

	/* Hold gt_lock across reset to prevent any register access
	* with forcewake not set correctly
	*/

	/* Take gt_lock to prevent register access*/
	spin_lock_irqsave(&dev_priv->gt_lock, irqflags);

	switch (ring->id) {
	case RCS:
		/* GEN6_GDRST is not in the gt power well, no need to check
		* for fifo space for the write or forcewake the chip for
		* the read
		*/
		I915_WRITE_NOTRACE(GEN6_GDRST, GEN6_GRDOM_RENDER);
		dev_priv->hangcheck[RCS].total++;

		/* Spin waiting for the device to ack the reset request */
		ret = wait_for_atomic_us((I915_READ_NOTRACE(GEN6_GDRST)
					& GEN6_GRDOM_RENDER) == 0, 500);
		DRM_DEBUG_TDR("RCS Reset\n");
		break;


	case BCS:
		I915_WRITE_NOTRACE(GEN6_GDRST, GEN6_GRDOM_BLT);
		dev_priv->hangcheck[BCS].total++;

		/* Spin waiting for the device to ack the reset request */
		ret = wait_for_atomic_us((I915_READ_NOTRACE(GEN6_GDRST)
				& GEN6_GRDOM_BLT) == 0, 500);
		DRM_DEBUG_TDR("BCS Reset\n");
		break;


	case VCS:
		I915_WRITE_NOTRACE(GEN6_GDRST, GEN6_GRDOM_MEDIA);
		dev_priv->hangcheck[VCS].total++;

		/* Spin waiting for the device to ack the reset request */
		ret = wait_for_atomic_us((I915_READ_NOTRACE(GEN6_GDRST)
					& GEN6_GRDOM_MEDIA) == 0, 500);
		DRM_DEBUG_TDR("VCS Reset\n");
		break;


	default:
		DRM_ERROR("Unexpected ring ID\n");
		break;
	}

	DRM_DEBUG_TDR("Reset result %d\n", ret);

	/* Request power management to restore the power state based
	* on the current reference count(s)*/
	gen6_gt_force_wake_restore(dev_priv);

	spin_unlock_irqrestore(&dev_priv->gt_lock, irqflags);

	/* Do uevent outside of spinlock as uevent can sleep */
	reset_event[0] = kasprintf(GFP_KERNEL, "RESET RING=%d", ring->id);
	kobject_uevent_env(&dev->primary->kdev.kobj,
			KOBJ_CHANGE, reset_event);

	return ret;
}


int intel_ring_save(struct intel_ring_buffer *ring, u32 flags)
{
	if (ring && ring->save)
		return ring->save(ring, ring->saved_state,
			I915_RING_CONTEXT_SIZE, flags);
	else {
		DRM_ERROR("ring save not supported\n");
		return -EINVAL;
	}
}

static int
gen6_ring_save(struct intel_ring_buffer *ring, uint32_t *data, uint32_t max,
		u32 flags)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	uint32_t idx = 0;
	uint32_t gen = INTEL_INFO(dev)->gen;
	uint32_t head;
	uint32_t tail;
	uint32_t head_addr;
	uint32_t tail_addr;
	int clamp_to_tail = 0;

	/* Ring save only added for gen >= 7 */
	WARN_ON(gen < 7);

	/* Save common registers */
	if (max < COMMON_RING_CTX_SIZE)
		return -EINVAL;

	head = I915_READ_HEAD(ring);
	tail = I915_READ_TAIL(ring);

	head_addr = head & HEAD_ADDR;
	tail_addr = tail & TAIL_ADDR;

	if (flags & FORCE_ADVANCE) {
		/* The head must always chase the tail.
		* If the tail is beyond the head then do not allow
		* the head to overtake it. If the tail is less than
		* the head then the tail has already wrapped and
		* there is no problem in advancing the head or even
		* wrapping the head back to 0 as worst case it will
		* become equal to tail */
		if (head_addr <= tail_addr)
			clamp_to_tail = 1;

		/* Force head to next QWORD boundary */
		head_addr &= ~0x7;
		head_addr += 8;

		if (clamp_to_tail && (head_addr > tail_addr)) {
			head_addr = tail_addr;
		} else if (head_addr >= ring->size) {
			/* Wrap head back to start if it exceeds ring size*/
			head_addr = 0;
		}

		/* Update the register */
		head &= ~HEAD_ADDR;
		head |= (head_addr & HEAD_ADDR);

		DRM_DEBUG_TDR("Forced head to 0x%08x\n", head);
	} else if (head & 0x7) {
		/* Ensure head pointer is pointing to a QWORD boundary */
		DRM_DEBUG_TDR("Rounding up head 0x%08x\n", head);
		head += 0x7;
		head &= ~0x7;
	}

	/* Saved with enable = 0 */
	data[idx++] = I915_READ_CTL(ring) & (RING_NR_PAGES | RING_REPORT_MASK);

	data[idx++] = (flags & RESET_HEAD_TAIL) ? 0 : tail;


	if (flags & RESET_HEAD_TAIL) {
		/* Save head as 0 so head is reset on restore */
		data[idx++] = 0;
	} else {
		/* Head will already have advanced to next instruction location
		* even if the current instruction caused a hang, so we just
		* save the current value as the value to restart at */
		data[idx++] = head;
	}

	data[idx++] = I915_READ_START(ring);

	/* Workaround for reading DCLV registers for gen < 8 */
	data[idx++] = (gen < 8) ?
			I915_READ(RING_PP_DIR_DCLV(&dev_priv->ring[VCS]))
			: I915_READ(RING_PP_DIR_DCLV(ring));
	data[idx++] = (gen < 8) ?
			 I915_READ(RING_PP_DIR_BASE(&dev_priv->ring[VCS]))
			: I915_READ(RING_PP_DIR_BASE(ring));

	switch (ring->id) {
	case RCS:
		if (max < (COMMON_RING_CTX_SIZE + RCS_RING_CTX_SIZE))
			return -EINVAL;

		data[idx++] = I915_READ(RENDER_HWS_PGA_GEN7);
		data[idx++] = I915_READ(RING_UHPTR(ring->mmio_base));
		data[idx++] = I915_READ(RING_INSTPM(ring->mmio_base));
		data[idx++] = I915_READ(RING_IMR(ring->mmio_base));
		data[idx++] = I915_READ(CACHE_MODE_0);
		data[idx++] = I915_READ(RING_MI_MODE(ring->mmio_base));
		data[idx++] = I915_READ(_3D_CHICKEN2);
		data[idx++] = I915_READ(_3D_CHICKEN3);
		data[idx++] = I915_READ(GAM_ECOCHK);
		data[idx++] = I915_READ(GFX_MODE_GEN7);
		data[idx++] = I915_READ(GEN6_RBSYNC);
		data[idx++] = I915_READ(GEN7_FF_THREAD_MODE);
		data[idx++] = I915_READ(RING_MAX_IDLE(ring->mmio_base));
		break;

	case VCS:
		if (max < (COMMON_RING_CTX_SIZE + VCS_RING_CTX_SIZE))
			return -EINVAL;

		data[idx++] = I915_READ(BSD_HWS_PGA_GEN7);
		data[idx++] = I915_READ(RING_MI_MODE(ring->mmio_base));
		data[idx++] = I915_READ(RING_IMR(ring->mmio_base));
		data[idx++] = I915_READ(RING_UHPTR(ring->mmio_base));
		data[idx++] = I915_READ(RING_INSTPM(ring->mmio_base));
		data[idx++] = I915_READ(RING_EXCC_GEN7(ring));
		data[idx++] = I915_READ(GAC_ECO_BITS);
		data[idx++] = I915_READ(RING_MODE_GEN7(ring));
		data[idx++] = I915_READ(GEN6_VRSYNC);
		data[idx++] = I915_READ(RING_MAX_IDLE(ring->mmio_base));
		break;

	case BCS:
		if (max < (COMMON_RING_CTX_SIZE + BCS_RING_CTX_SIZE))
			return -EINVAL;

		data[idx++] = I915_READ(BLT_HWS_PGA_GEN7);
		data[idx++] = I915_READ(RING_MI_MODE(ring->mmio_base));
		data[idx++] = I915_READ(RING_IMR(ring->mmio_base));
		data[idx++] = I915_READ(RING_UHPTR(ring->mmio_base));
		data[idx++] = I915_READ(RING_INSTPM(ring->mmio_base));
		data[idx++] = I915_READ(RING_EXCC_GEN7(ring));
		data[idx++] = I915_READ(GAB_CTL);
		data[idx++] = I915_READ(RING_MODE_GEN7(ring));
		data[idx++] = I915_READ(GEN6_BRSYNC);
		data[idx++] = I915_READ(GEN6_BVSYNC);
		data[idx++] = I915_READ(RING_MAX_IDLE(ring->mmio_base));
		break;
	}

	return 0;
}


int intel_ring_restore(struct intel_ring_buffer *ring)
{
	if (ring && ring->restore)
		return ring->restore(ring, ring->saved_state,
				I915_RING_CONTEXT_SIZE);
	else {
		DRM_ERROR("ring restore not supported\n");
		return -EINVAL;
	}
}


static int
gen6_ring_restore(struct intel_ring_buffer *ring, uint32_t *data,
			uint32_t max)
{
	struct drm_device *dev = ring->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	uint32_t idx = 0;
	uint32_t x;

	/* NOTE: Registers are restored in reverse order from when
	*        they were saved. */
	switch (ring->id) {
	case RCS:
		if (max < (COMMON_RING_CTX_SIZE + RCS_RING_CTX_SIZE))
			return -EINVAL;

		idx = COMMON_RING_CTX_SIZE + RCS_RING_CTX_SIZE - 1;

		I915_WRITE(RING_MAX_IDLE(ring->mmio_base), data[idx--]);
		I915_WRITE(GEN7_FF_THREAD_MODE, data[idx--]);
		I915_WRITE(GEN6_RBSYNC, data[idx--]);
		I915_WRITE(RING_MODE_GEN7(ring), data[idx--]);
		I915_WRITE(GAM_ECOCHK, data[idx--]);
		I915_WRITE(_3D_CHICKEN3, data[idx--]);
		I915_WRITE(_3D_CHICKEN2, data[idx--]);
		I915_WRITE(RING_MI_MODE(ring->mmio_base), data[idx--]);
		I915_WRITE(CACHE_MODE_0, data[idx--]);
		I915_WRITE(RING_IMR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_INSTPM(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_UHPTR(ring->mmio_base), data[idx--]);
		I915_WRITE(RENDER_HWS_PGA_GEN7, data[idx--]);
		break;

	case VCS:
		if (max < (COMMON_RING_CTX_SIZE + VCS_RING_CTX_SIZE))
			return -EINVAL;

		idx = COMMON_RING_CTX_SIZE + VCS_RING_CTX_SIZE - 1;

		I915_WRITE(RING_MAX_IDLE(ring->mmio_base), data[idx--]);
		I915_WRITE(GEN6_VRSYNC, data[idx--]);
		I915_WRITE(RING_MODE_GEN7(ring), data[idx--]);
		I915_WRITE(GAC_ECO_BITS, data[idx--]);
		I915_WRITE(RING_EXCC_GEN7(ring), data[idx--]);
		I915_WRITE(RING_INSTPM(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_UHPTR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_IMR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_MI_MODE(ring->mmio_base), data[idx--]);
		I915_WRITE(BSD_HWS_PGA_GEN7, data[idx--]);
		break;

	case BCS:
		if (max < (COMMON_RING_CTX_SIZE + BCS_RING_CTX_SIZE))
			return -EINVAL;

		idx = COMMON_RING_CTX_SIZE + BCS_RING_CTX_SIZE - 1;

		I915_WRITE(RING_MAX_IDLE(ring->mmio_base), data[idx--]);
		I915_WRITE(GEN6_BVSYNC, data[idx--]);
		I915_WRITE(GEN6_BRSYNC, data[idx--]);
		I915_WRITE(RING_MODE_GEN7(ring), data[idx--]);
		I915_WRITE(GAB_CTL, data[idx--]);
		I915_WRITE(RING_EXCC_GEN7(ring), data[idx--]);
		I915_WRITE(RING_INSTPM(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_UHPTR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_IMR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_MI_MODE(ring->mmio_base), data[idx--]);
		I915_WRITE(BLT_HWS_PGA_GEN7, data[idx--]);
		break;
	}

	/* Restore common registers */
	if (max < COMMON_RING_CTX_SIZE)
		return -EINVAL;

	idx = COMMON_RING_CTX_SIZE - 1;

	I915_WRITE(RING_PP_DIR_BASE(ring), data[idx--]);
	I915_WRITE(RING_PP_DIR_DCLV(ring), data[idx--]);

	/* Write ring base address before head/tail as it clears head to 0 */
	I915_WRITE_START(ring, data[idx--]);
	x = I915_READ_START(ring);
	I915_WRITE_HEAD(ring, data[idx--]);
	I915_WRITE_TAIL(ring, data[idx--]);
	I915_WRITE_CTL(ring, data[idx--]);

	return 0;
}



int intel_init_render_ring_buffer(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring = &dev_priv->ring[RCS];

	ring->name = "render ring";
	ring->id = RCS;
	ring->mmio_base = RENDER_RING_BASE;

	if (INTEL_INFO(dev)->gen >= 6) {
		ring->add_request = gen6_add_request;
		ring->flush = gen6_render_ring_flush;
		if (INTEL_INFO(dev)->gen == 6)
			ring->flush = gen6_render_ring_flush__wa;
		ring->irq_get = gen6_ring_get_irq;
		ring->irq_put = gen6_ring_put_irq;
		ring->irq_enable_mask = GT_USER_INTERRUPT;
		ring->get_seqno = gen6_ring_get_seqno;
		ring->sync_to = gen6_ring_sync;
		ring->enable = gen6_ring_enable;
		ring->disable = gen6_ring_disable;
		ring->reset = gen6_ring_reset;
		ring->save = gen6_ring_save;
		ring->restore = gen6_ring_restore;
		ring->semaphore_register[0] = MI_SEMAPHORE_SYNC_INVALID;
		ring->semaphore_register[1] = MI_SEMAPHORE_SYNC_RV;
		ring->semaphore_register[2] = MI_SEMAPHORE_SYNC_RB;
		ring->signal_mbox[0] = GEN6_VRSYNC;
		ring->signal_mbox[1] = GEN6_BRSYNC;
	} else if (IS_GEN5(dev)) {
		ring->add_request = pc_render_add_request;
		ring->flush = gen4_render_ring_flush;
		ring->get_seqno = pc_render_get_seqno;
		ring->irq_get = gen5_ring_get_irq;
		ring->irq_put = gen5_ring_put_irq;
		ring->irq_enable_mask = GT_USER_INTERRUPT | GT_PIPE_NOTIFY;
	} else {
		ring->add_request = i9xx_add_request;
		if (INTEL_INFO(dev)->gen < 4)
			ring->flush = gen2_render_ring_flush;
		else
			ring->flush = gen4_render_ring_flush;
		ring->get_seqno = ring_get_seqno;
		if (IS_GEN2(dev)) {
			ring->irq_get = i8xx_ring_get_irq;
			ring->irq_put = i8xx_ring_put_irq;
		} else {
			ring->irq_get = i9xx_ring_get_irq;
			ring->irq_put = i9xx_ring_put_irq;
		}
		ring->irq_enable_mask = I915_USER_INTERRUPT;
	}
	ring->write_tail = ring_write_tail;
	if (INTEL_INFO(dev)->gen >= 6)
		ring->dispatch_execbuffer = gen6_ring_dispatch_execbuffer;
	else if (INTEL_INFO(dev)->gen >= 4)
		ring->dispatch_execbuffer = i965_dispatch_execbuffer;
	else if (IS_I830(dev) || IS_845G(dev))
		ring->dispatch_execbuffer = i830_dispatch_execbuffer;
	else
		ring->dispatch_execbuffer = i915_dispatch_execbuffer;
	ring->init = init_render_ring;
	ring->cleanup = render_ring_cleanup;


	if (!I915_NEED_GFX_HWS(dev)) {
		ring->status_page.page_addr = dev_priv->status_page_dmah->vaddr;
		memset(ring->status_page.page_addr, 0, PAGE_SIZE);
	}

	return intel_init_ring_buffer(dev, ring);
}

int intel_render_ring_init_dri(struct drm_device *dev, u64 start, u32 size)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring = &dev_priv->ring[RCS];

	ring->name = "render ring";
	ring->id = RCS;
	ring->mmio_base = RENDER_RING_BASE;

	if (INTEL_INFO(dev)->gen >= 6) {
		/* non-kms not supported on gen6+ */
		return -ENODEV;
	}

	/* Note: gem is not supported on gen5/ilk without kms (the corresponding
	 * gem_init ioctl returns with -ENODEV). Hence we do not need to set up
	 * the special gen5 functions. */
	ring->add_request = i9xx_add_request;
	if (INTEL_INFO(dev)->gen < 4)
		ring->flush = gen2_render_ring_flush;
	else
		ring->flush = gen4_render_ring_flush;
	ring->get_seqno = ring_get_seqno;
	if (IS_GEN2(dev)) {
		ring->irq_get = i8xx_ring_get_irq;
		ring->irq_put = i8xx_ring_put_irq;
	} else {
		ring->irq_get = i9xx_ring_get_irq;
		ring->irq_put = i9xx_ring_put_irq;
	}
	ring->irq_enable_mask = I915_USER_INTERRUPT;
	ring->write_tail = ring_write_tail;
	if (INTEL_INFO(dev)->gen >= 4)
		ring->dispatch_execbuffer = i965_dispatch_execbuffer;
	else if (IS_I830(dev) || IS_845G(dev))
		ring->dispatch_execbuffer = i830_dispatch_execbuffer;
	else
		ring->dispatch_execbuffer = i915_dispatch_execbuffer;
	ring->init = init_render_ring;
	ring->cleanup = render_ring_cleanup;

	if (!I915_NEED_GFX_HWS(dev))
		ring->status_page.page_addr = dev_priv->status_page_dmah->vaddr;

	ring->dev = dev;
	INIT_LIST_HEAD(&ring->active_list);
	INIT_LIST_HEAD(&ring->request_list);

	ring->size = size;
	ring->effective_size = ring->size;
	if (IS_I830(ring->dev))
		ring->effective_size -= 128;

	ring->virtual_start = ioremap_wc(start, size);
	if (ring->virtual_start == NULL) {
		DRM_ERROR("can not ioremap virtual address for"
			  " ring buffer\n");
		return -ENOMEM;
	}

	return 0;
}

int intel_init_bsd_ring_buffer(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring = &dev_priv->ring[VCS];

	ring->name = "bsd ring";
	ring->id = VCS;

	ring->write_tail = ring_write_tail;
	if (IS_GEN6(dev) || IS_GEN7(dev)) {
		ring->mmio_base = GEN6_BSD_RING_BASE;
		/* gen6 bsd needs a special wa for tail updates */
		if (IS_GEN6(dev))
			ring->write_tail = gen6_bsd_ring_write_tail;
		ring->flush = gen6_ring_flush;
		ring->add_request = gen6_add_request;
		ring->get_seqno = gen6_ring_get_seqno;
		ring->irq_enable_mask = GEN6_BSD_USER_INTERRUPT;
		ring->irq_get = gen6_ring_get_irq;
		ring->irq_put = gen6_ring_put_irq;
		ring->dispatch_execbuffer = gen6_ring_dispatch_execbuffer;
		ring->sync_to = gen6_ring_sync;
		ring->enable = gen6_ring_enable;
		ring->disable = gen6_ring_disable;
		ring->reset = gen6_ring_reset;
		ring->save = gen6_ring_save;
		ring->restore = gen6_ring_restore;
		ring->semaphore_register[0] = MI_SEMAPHORE_SYNC_VR;
		ring->semaphore_register[1] = MI_SEMAPHORE_SYNC_INVALID;
		ring->semaphore_register[2] = MI_SEMAPHORE_SYNC_VB;
		ring->signal_mbox[0] = GEN6_RVSYNC;
		ring->signal_mbox[1] = GEN6_BVSYNC;
	} else {
		ring->mmio_base = BSD_RING_BASE;
		ring->flush = bsd_ring_flush;
		ring->add_request = i9xx_add_request;
		ring->get_seqno = ring_get_seqno;
		if (IS_GEN5(dev)) {
			ring->irq_enable_mask = GT_BSD_USER_INTERRUPT;
			ring->irq_get = gen5_ring_get_irq;
			ring->irq_put = gen5_ring_put_irq;
		} else {
			ring->irq_enable_mask = I915_BSD_USER_INTERRUPT;
			ring->irq_get = i9xx_ring_get_irq;
			ring->irq_put = i9xx_ring_put_irq;
		}
		ring->dispatch_execbuffer = i965_dispatch_execbuffer;
	}
	ring->init = init_ring_common;

	/* Enable the timeout counter for watchdog reset */
	I915_WRITE_IMR(ring, ~GEN6_BSD_TIMEOUT_COUNTER_EXPIRED);

	return intel_init_ring_buffer(dev, ring);
}

int intel_init_blt_ring_buffer(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring = &dev_priv->ring[BCS];

	ring->name = "blitter ring";
	ring->id = BCS;

	ring->mmio_base = BLT_RING_BASE;
	ring->write_tail = ring_write_tail;
	ring->flush = blt_ring_flush;
	ring->add_request = gen6_add_request;
	ring->get_seqno = gen6_ring_get_seqno;
	ring->irq_enable_mask = GEN6_BLITTER_USER_INTERRUPT;
	ring->irq_get = gen6_ring_get_irq;
	ring->irq_put = gen6_ring_put_irq;
	ring->dispatch_execbuffer = gen6_ring_dispatch_execbuffer;
	ring->sync_to = gen6_ring_sync;
	ring->enable = gen6_ring_enable;
	ring->disable = gen6_ring_disable;
	ring->reset = gen6_ring_reset;
	ring->save = gen6_ring_save;
	ring->restore = gen6_ring_restore;
	ring->semaphore_register[0] = MI_SEMAPHORE_SYNC_BR;
	ring->semaphore_register[1] = MI_SEMAPHORE_SYNC_BV;
	ring->semaphore_register[2] = MI_SEMAPHORE_SYNC_INVALID;
	ring->signal_mbox[0] = GEN6_RBSYNC;
	ring->signal_mbox[1] = GEN6_VBSYNC;
	ring->init = init_ring_common;

	return intel_init_ring_buffer(dev, ring);
}

int
intel_ring_flush_all_caches(struct intel_ring_buffer *ring)
{
	int ret, i;

	if (!ring->gpu_caches_dirty)
		return 0;

	ret = ring->flush(ring, 0, I915_GEM_GPU_DOMAINS);
	if (ret)
		return ret;

	/* WaReadAfterWriteHazard*/
	/* Send a number of Store Data commands here to finish
	   flushing hardware pipeline.This is needed in the case
	   where the next workload tries reading from the same
	   surface that this batch writes to. Without these StoreDWs,
	   not all of the data will actually be flushd to the surface
	   by the time the next batch starts reading it, possibly
	   causing a small amount of corruption.*/
	ret = intel_ring_begin(ring, 4 * 12);
	if (ret)
		return ret;
	for (i = 0; i < 12; i++) {
		intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
		intel_ring_emit(ring, I915_GEM_SCRATCH_INDEX <<
						MI_STORE_DWORD_INDEX_SHIFT);
		intel_ring_emit(ring, 0);
		intel_ring_emit(ring, MI_NOOP);
	}
	intel_ring_advance(ring);

	trace_i915_gem_ring_flush(ring, 0, I915_GEM_GPU_DOMAINS);

	ring->gpu_caches_dirty = false;
	return 0;
}

int
intel_ring_invalidate_all_caches(struct intel_ring_buffer *ring)
{
	uint32_t flush_domains;
	int ret, i;

	/* WaTlbInvalidateStoreDataBefore*/
	/* Before pipecontrol with TLB invalidate set, need 2 store
	   data commands (such as MI_STORE_DATA_IMM or MI_STORE_DATA_INDEX)
	   Without this, hardware cannot guarantee the command after the
	   PIPE_CONTROL with TLB inv will not use the old TLB values.*/
	ret = intel_ring_begin(ring, 4 * 2);
	if (ret)
		return ret;
	for (i = 0; i < 2; i++) {
		intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
		intel_ring_emit(ring, I915_GEM_SCRATCH_INDEX <<
						MI_STORE_DWORD_INDEX_SHIFT);
		intel_ring_emit(ring, 0);
		intel_ring_emit(ring, MI_NOOP);
	}
	intel_ring_advance(ring);

	flush_domains = 0;
	if (ring->gpu_caches_dirty)
		flush_domains = I915_GEM_GPU_DOMAINS;

	ret = ring->flush(ring, I915_GEM_GPU_DOMAINS, flush_domains);
	if (ret)
		return ret;

	trace_i915_gem_ring_flush(ring, I915_GEM_GPU_DOMAINS, flush_domains);

	ring->gpu_caches_dirty = false;
	return 0;
}

int
intel_ring_supports_watchdog(struct intel_ring_buffer *ring)
{
	/* Return 1 if the ring supports watchdog reset, otherwise 0 */
	if (ring) {
		switch (ring->id) {
		case RCS:
		case VCS:
			return 1;
		default:
			return 0;
		}
	}

	return 0;
}


int
intel_ring_start_watchdog(struct intel_ring_buffer *ring)
{
	int ret;
	drm_i915_private_t *dev_priv = ring->dev->dev_private;

	ret = intel_ring_begin(ring, 10);
	if (ret)
		return ret;

	/* i915_reg.h includes a warning to place a MI_NOOP
	* before a MI_LOAD_REGISTER_IMM*/
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_emit(ring, MI_NOOP);

	/* Set counter period */
	intel_ring_emit(ring, MI_LOAD_REGISTER_IMM(1));
	intel_ring_emit(ring, RING_THRESH(ring->mmio_base));
	intel_ring_emit(ring, dev_priv->watchdog_threshold[ring->id]);
	intel_ring_emit(ring, MI_NOOP);

	/* Start counter */
	intel_ring_emit(ring, MI_LOAD_REGISTER_IMM(1));
	intel_ring_emit(ring, RING_CNTR(ring->mmio_base));
	intel_ring_emit(ring, WATCHDOG_ENABLE);
	intel_ring_emit(ring, MI_NOOP);

	intel_ring_advance(ring);

	return 0;
}


int
intel_ring_stop_watchdog(struct intel_ring_buffer *ring)
{
	int ret;

	ret = intel_ring_begin(ring, 6);
	if (ret)
		return ret;

	/* i915_reg.h includes a warning to place a MI_NOOP
	* before a MI_LOAD_REGISTER_IMM*/
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_emit(ring, MI_NOOP);

	intel_ring_emit(ring, MI_LOAD_REGISTER_IMM(1));
	intel_ring_emit(ring, RING_CNTR(ring->mmio_base));

	switch (ring->id) {
	default:
	case RCS:
		intel_ring_emit(ring, RCS_WATCHDOG_DISABLE);
		break;
	case VCS:
		intel_ring_emit(ring, VCS_WATCHDOG_DISABLE);
		break;
	}

	intel_ring_emit(ring, MI_NOOP);

	intel_ring_advance(ring);

	return 0;
}




