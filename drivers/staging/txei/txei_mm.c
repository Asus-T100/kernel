#define pr_fmt(fmt) "mm :" fmt

#define DEBUG
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/compat.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include "txei_dev.h"
#include "txei_mm.h"



#ifndef __phys_to_pfn
#define __phys_to_pfn(phys) ((phys) >> PAGE_SHIFT)
#endif

#ifndef VM_DONTDUMP
#define VM_DONTDUMP VM_RESERVED
#endif

struct txei_mm_pool {
	void *vaddr;
	phys_addr_t paddr;
	size_t size;
	size_t offset;
};

struct txei_mm_client {
	size_t size;
	phys_addr_t paddr;
	bool q;
};
struct txei_mm_device {
	struct miscdevice dev;
	struct mutex lock;
	struct txei_mm_client client;
	struct dentry *debug_root;
	struct txei_mm_pool pool;
};


#define txeimm_warn(__dev, fmt, ...) \
	dev_warn((__dev)->dev.parent, "Warn: %s[%d]: " pr_fmt(fmt),  \
		__func__, __LINE__, ##__VA_ARGS__)

#define txeimm_info(__dev, fmt, ...) \
	dev_info((__dev)->dev.parent, "Info: " pr_fmt(fmt), ##__VA_ARGS__)

#define txeimm_err(__dev, fmt, ...) \
	dev_err((__dev)->dev.parent, "Error: " pr_fmt(fmt), ##__VA_ARGS__)

#define txeimm_dbg(__dev, fmt, ...) \
	dev_dbg((__dev)->dev.parent, "%s[%d]: " pr_fmt(fmt), \
		__func__, __LINE__, ##__VA_ARGS__)

/**
 * txei_mm_open - the open function
 *
 * @inode: pointer to inode structure
 * @file: pointer to file structure
 *
 * returns 0 on success, <0 on error
 */
static int txei_mm_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct txei_mm_device *mdev =
			container_of(miscdev, struct txei_mm_device, dev);

	mutex_lock(&mdev->lock);
	if (mdev->client.q) {
		mutex_unlock(&mdev->lock);
		return -EBUSY;
	}
	mdev->client.q = true;
	mutex_unlock(&mdev->lock);
	file->private_data = &mdev->client;

	txeimm_dbg(mdev, "device opened\n");

	return nonseekable_open(inode, file);

}

/**
 * txei_mm_release - the release function
 *
 * @inode: pointer to inode structure
 * @file: pointer to file structure
 *
 * returns 0 on success, <0 on error
 */
static int txei_mm_release(struct inode *inode, struct file *file)
{
	struct txei_mm_client *client = file->private_data;
	struct txei_mm_device *mdev =
			container_of(client, struct txei_mm_device, client);
	size_t len =  client->size;

	mutex_lock(&mdev->lock);

	if (!client->q)
		goto out;

	if (mdev->pool.offset < len)  {
		txeimm_info(mdev, "pool smaller then requested size...truncating");
		len = mdev->pool.offset;
	}
	mdev->pool.offset -= len;

	client->q = false;
	txeimm_info(mdev, "txeimm: release %zd\n", len);
out:
	mutex_unlock(&mdev->lock);
	return 0;
}
/**
 * txei_mm_distribute_chunk - destributing memory chunk to
 * Sec Application (shim library)
 * Chunks are taken from a pool of DMA allocated memory
 * each time the user space application does the ioctl call
 * for mm allocate.
 * @file: pointer to file structure
 * @cmd: ioctl command
 * @data: pointer to shim memory structure
 *
 * returns 0 on success , <0 on error
 */

static int txei_mm_distribute_chunk(struct txei_mm_device *mdev,
	unsigned long arg)
{

	struct txei_mm_data req;
	int ret;
	size_t aligned_size; /* for mmap to work we need page multipliers */

	if (copy_from_user(&req, (char __user *)arg, sizeof(req))) {
		ret = -EFAULT;
		txeimm_err(mdev, "EFAULT");
		goto err;
	}

	aligned_size = PAGE_ALIGN(req.size);

	/* Lock here because more than one entity can accesss pool */
	mutex_lock(&mdev->lock);
	if (aligned_size > mdev->pool.size - mdev->pool.offset) {
		txeimm_err(mdev, ":can't allocate mem from chunk: %zd > %zd - %zd\n",
			aligned_size, mdev->pool.size, mdev->pool.offset);
		mutex_unlock(&mdev->lock);
		ret = -ENOMEM;
		goto err_unlock;
	}

	req.vaddr = (uintptr_t)mdev->pool.vaddr + mdev->pool.offset;
	req.paddr = mdev->pool.paddr + mdev->pool.offset;
	req.size = aligned_size;

	txeimm_info(mdev, "Allocate mem from chunk: vaddr=%llu paddr=%llu size=%llu\n",
		 req.vaddr, req.paddr, req.size);

	mdev->pool.offset += aligned_size;
	if (copy_to_user((char __user *)arg, &req, sizeof(req))) {
		ret = -EFAULT;
		pr_err("txei_mm: EFAULT, %s, %d\n", __FILE__, __LINE__);
		mdev->pool.offset -= aligned_size;
		mutex_unlock(&mdev->lock);
		goto err;
	}
	mdev->client.size = aligned_size;
	mdev->client.paddr = req.paddr;
	mutex_unlock(&mdev->lock);
	return 0;

err_unlock:
	mutex_unlock(&mdev->lock);
err:
	return ret;
}

/**
 * txei_mm_free - returns memory chunk from Sec Application (shim library
 *
 * @file: pointer to file structure
 * @cmd: ioctl command
 * @data: pointer to shim memory structure
 *
 * returns 0 on success , <0 on error
 */
static int txei_mm_free(struct txei_mm_device *mdev, unsigned long arg)
{
	struct txei_mm_data req;
	int ret;

	if (copy_from_user(&req, (char __user *)arg, sizeof(req))) {
		ret = -EFAULT;
		txeimm_err(mdev, "EFAULT\n");
		goto err;
	}
	mutex_lock(&mdev->lock);
	/* FIXME: validate free */
	mdev->pool.offset -= req.size;
	mdev->client.size = 0;
	mdev->client.paddr = 0LL;
	mutex_unlock(&mdev->lock);
	return 0;
err:
	return ret;

}

/**
 * txei_ioctl - the IOCTL function
 *
 * @file: pointer to file structure
 * @cmd: ioctl command
 * @data: pointer to txei message structure
 *
 * returns 0 on success , <0 on error
 */

static long txei_mm_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct txei_mm_client *client = file->private_data;
	struct txei_mm_device *mdev =
			container_of(client, struct txei_mm_device, client);
	int ret;

	/* don't even decode wrong cmds: better returning  ENOTTY than EFAULT */
	if (_IOC_TYPE(cmd) != 'H') {
		txeimm_err(mdev, "Wrong IOCTL type: Got %c wanted %c\n",
			_IOC_TYPE(cmd), 'H');
		ret = -ENOTTY;
		goto out;
	}
	if (_IOC_NR(cmd) > TXEI_IOC_MAXNR) {
		txeimm_err(mdev, "%s: Wrong IOCTL num. Got %d wanted max %d\n",
			"txei_mm", _IOC_NR(cmd), TXEI_IOC_MAXNR);
		ret = -ENOTTY;
		goto out;
	}
	if (!access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd))) {
		ret = -EFAULT;
		goto out;
	}

	switch (cmd) {
	case IOCTL_TXEI_MM_ALLOC:
		txeimm_dbg(mdev, "IOCTL_TXEI_ALLOC_MEM_CALL\n");
		ret = txei_mm_distribute_chunk(mdev, arg);
	break;
	case IOCTL_TXEI_MM_FREE:
		txeimm_dbg(mdev, "IOCTL_TXEI_FREE_MEM_CALL\n");
		ret = txei_mm_free(mdev, arg);
	break;
	default:
		ret = -EINVAL;
		txeimm_err(mdev, "Invalid IOCTL command %d\n", cmd);
		break;
	}
out:
	return ret;
}

static int txei_mm_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct txei_mm_client *client = file->private_data;
	struct txei_mm_device *mdev =
			container_of(client, struct txei_mm_device, client);
	size_t vsize = vma->vm_end - vma->vm_start;
	size_t off = vma->vm_pgoff << PAGE_SHIFT;
	int ret;

	txeimm_dbg(mdev, "vm_start=0x%016lX vm_end=0x%016lX vm_pgoff=0x%016lX off=%zd\n",
		vma->vm_start,  vma->vm_end, vma->vm_pgoff, off);

	mutex_lock(&mdev->lock);
	if (vsize > client->size || off > client->paddr + client->size) {
		txeimm_err(mdev, "%s: trying to map larger area than available r.size=%zd a.size=%zd vm->pg_off=%ld\n",
			__func__, vsize, client->size, vma->vm_pgoff);
		ret = -EINVAL;
		goto err;
	}


	vma->vm_flags |= VM_DONTDUMP | VM_READ | VM_WRITE | VM_SHARED |
		VM_DONTEXPAND;
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	if (remap_pfn_range(vma, vma->vm_start,
			__phys_to_pfn(client->paddr) + vma->vm_pgoff,
			vsize, vma->vm_page_prot)) {
		ret = -EAGAIN;
		goto err;
	}

	mutex_unlock(&mdev->lock);
	vma->vm_private_data = client;
	return 0;

err:
	mutex_unlock(&mdev->lock);
	return ret;
}

static int txei_mm_dbgfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return nonseekable_open(inode, file);
}

static ssize_t txei_mm_dbgfs_pool_read(struct file *file,
			char __user *user_buf,
			size_t count, loff_t *ppos)
{

	struct txei_mm_device *mdev = file->private_data;
	return simple_read_from_buffer(user_buf, count, ppos,
				mdev->pool.vaddr, 256);
}
static const struct file_operations txei_mm_dbgfs_pool_ops = {
	.read = txei_mm_dbgfs_pool_read,
	.open = txei_mm_dbgfs_open,
	.llseek = generic_file_llseek,                                  \
};


/**
 * txei_compat_ioctl - the compat IOCTL function
 *
 * @file: pointer to file structure
 * @cmd: ioctl command
 * @data: pointer to txei message structure
 *
 * returns 0 on success , <0 on error
 */
#ifdef CONFIG_COMPAT
static long txei_mm_compat_ioctl(struct file *file,
		      unsigned int cmd, unsigned long data)
{
	return txei_mm_ioctl(file, cmd, (unsigned long)compat_ptr(data));
}
#endif
/*
 * file operations structure will be used for txei char device.
 */
static const struct file_operations txei_mm_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = txei_mm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = txei_mm_compat_ioctl,
#endif /* CONFIG_COMPAT */
	.open = txei_mm_open,
	.release = txei_mm_release,
	.mmap = txei_mm_mmap,
	/*	.poll = txei_mmpoll,*/
	.llseek = no_llseek
};

struct txei_mm_device *txei_mm_init(struct device *dev, void *vaddr,
	dma_addr_t paddr, size_t size)
{

	struct txei_mm_device *mdev;
	int ret;

	mdev = kzalloc(sizeof(struct txei_mm_device), GFP_KERNEL);
	if (!mdev)
		return ERR_PTR(-ENOMEM);

	mdev->dev.minor = MISC_DYNAMIC_MINOR;
	mdev->dev.name = "txeimm";
	mdev->dev.fops = &txei_mm_fops;
	mdev->dev.parent = dev;
	/* init pci module */
	ret = misc_register(&mdev->dev);
	if (ret) {
		kfree(mdev);
		dev_err(dev, "cant't register misc device.\n") ;
		return ERR_PTR(ret);
	}

	mutex_init(&mdev->lock);

	mdev->pool.vaddr = vaddr;
	mdev->pool.paddr = paddr;
	mdev->pool.size = size;

#ifdef CONFIG_DEBUG_FS
	mdev->debug_root = debugfs_create_dir("txeimm", NULL);
	if (IS_ERR_OR_NULL(mdev->debug_root))
		txeimm_err(mdev, "failed to create debug files.\n");

	else
		debugfs_create_file("pool", S_IRUSR, mdev->debug_root,
			mdev, &txei_mm_dbgfs_pool_ops);
#endif /* CONFIG_DEBUG_FS */

	return mdev;
}



/**
 * txei_dma_deinit - De-Init Routine for txei_dma misc device
 *
 * txei_dma_deinit is called by release function of txei module.
 */
void txei_mm_deinit(struct txei_mm_device *mdev)
{
	if (mdev == NULL)
		return;

#ifdef CONFIG_DEBUG_FS
	if (mdev->debug_root)
		debugfs_remove_recursive(mdev->debug_root);
	mdev->debug_root = NULL;
#endif /* CONFIG_DEBUG_FS */

	misc_deregister(&mdev->dev);
	kfree(mdev);
	pr_debug("txei: Driver unloaded successfully.\n");
}


