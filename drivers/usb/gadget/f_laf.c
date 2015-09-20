/*
 * Gadget Driver for Android LAF
 *
 * Copyright (C) 2012-2015 LG Electronics, Inc.
 * Author: DH, kang <deunghyung.kang@lge.com>
 *         Hansun Lee <hansun.lee@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/file.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

//#define LAF_DEBUG

#ifdef LAF_DEBUG
#define LAF_DBG(d, fmt, args...) INFO(d, fmt, ## args)
#define LAF_VDBG(d, fmt, args...) INFO(d, fmt, ## args)
#else
#define LAF_DBG(d, fmt, args...)
#define LAF_VDBG(d, fmt, args...)
#endif

#define LAF_BULK_BUFFER_SIZE 16384

/* values for laf_dev.state */
#define LAF_STATE_OFFLINE               0x01   /* initial state, disconnected */
#define LAF_STATE_READY                 0x02   /* ready for userspace calls */
#define LAF_STATE_WRITE                 0x04   /* processing userspace write call */
#define LAF_STATE_READ                  0x08   /* processing userspace read call */
#define LAF_STATE_ERROR                 0x10   /* error from completion routine */

/* number of tx and rx requests to allocate */
#define LAF_TX_REQ_MAX 8
#define LAF_RX_REQ_MAX 8

unsigned int laf_tx_req_len = LAF_BULK_BUFFER_SIZE;
module_param(laf_tx_req_len, uint, S_IRUGO | S_IWUSR);

unsigned int laf_rx_req_len = LAF_BULK_BUFFER_SIZE;
module_param(laf_rx_req_len, uint, S_IRUGO | S_IWUSR);

unsigned int laf_tx_reqs = LAF_TX_REQ_MAX;
module_param(laf_tx_reqs, uint, S_IRUGO | S_IWUSR);

unsigned int laf_rx_reqs = LAF_RX_REQ_MAX;
module_param(laf_rx_reqs, uint, S_IRUGO | S_IWUSR);

struct laf_dev {
	struct usb_function function;

	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	int state;

	/* synchronize access to our device file */
	atomic_t open_excl;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_pool;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	struct usb_request *rx_req[LAF_RX_REQ_MAX];
};

static struct usb_interface_descriptor laf_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass     = USB_SUBCLASS_VENDOR_SPEC,
	.bInterfaceProtocol     = 0,
};

static struct usb_endpoint_descriptor laf_superspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor laf_superspeed_in_comp_desc = {
	.bLength =		sizeof laf_superspeed_in_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor laf_superspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor laf_superspeed_out_comp_desc = {
	.bLength =		sizeof laf_superspeed_out_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor laf_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor laf_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor laf_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor laf_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_laf_descs[] = {
	(struct usb_descriptor_header *) &laf_interface_desc,
	(struct usb_descriptor_header *) &laf_fullspeed_in_desc,
	(struct usb_descriptor_header *) &laf_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_laf_descs[] = {
	(struct usb_descriptor_header *) &laf_interface_desc,
	(struct usb_descriptor_header *) &laf_highspeed_in_desc,
	(struct usb_descriptor_header *) &laf_highspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *ss_laf_descs[] = {
	(struct usb_descriptor_header *) &laf_interface_desc,
	(struct usb_descriptor_header *) &laf_superspeed_in_desc,
	(struct usb_descriptor_header *) &laf_superspeed_in_comp_desc,
	(struct usb_descriptor_header *) &laf_superspeed_out_desc,
	(struct usb_descriptor_header *) &laf_superspeed_out_comp_desc,
	NULL,
};

/* temporary variable used between laf_open() and laf_gadget_bind() */
static struct laf_dev *_laf_dev;

static inline struct laf_dev *func_to_laf(struct usb_function *f)
{
	return container_of(f, struct laf_dev, function);
}

static struct usb_request *laf_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void laf_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int laf_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void laf_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
void laf_req_put(struct laf_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
struct usb_request *laf_req_get(struct laf_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void laf_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct laf_dev *dev = _laf_dev;

	LAF_VDBG(dev->cdev, "%s: req->status(%d)\n", __func__, req->status);

	if (req->status != 0 && dev->state != LAF_STATE_OFFLINE)
		dev->state = LAF_STATE_ERROR;

	laf_req_put(dev, &dev->tx_idle, req);

	wake_up(&dev->write_wq);
}

static void laf_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct laf_dev *dev = _laf_dev;

	LAF_VDBG(dev->cdev, "%s: req->status(%d)\n", __func__, req->status);

	if (req->status != 0 && dev->state != LAF_STATE_OFFLINE)
		dev->state = LAF_STATE_ERROR;

	if (req->status != 0) {
		laf_req_put(dev, &dev->rx_idle, req);
	} else {
		laf_req_put(dev, &dev->rx_pool, req);
	}

	wake_up(&dev->read_wq);
}

static int laf_create_bulk_endpoints(struct laf_dev *dev,
		struct usb_endpoint_descriptor *in_desc,
		struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	LAF_DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		LAF_DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	LAF_DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		LAF_DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	LAF_DBG(cdev, "usb_ep_autoconfig for laf ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	/* now allocate requests for our endpoints */
retry_tx_alloc:
	for (i = 0; i < laf_tx_reqs; i++) {
		req = laf_request_new(dev->ep_in, laf_tx_req_len);
		if (!req) {
			while ((req = laf_req_get(dev, &dev->tx_idle)))
				laf_request_free(req, dev->ep_in);

			if (laf_tx_req_len <= LAF_BULK_BUFFER_SIZE) {
				pr_err("%s: tx_req_fail\n", __func__);
				goto tx_req_fail;
			}

			laf_tx_req_len = LAF_BULK_BUFFER_SIZE;
			laf_tx_reqs = LAF_TX_REQ_MAX;
			goto retry_tx_alloc;
		}
		req->complete = laf_complete_in;
		laf_req_put(dev, &dev->tx_idle, req);
	}

	if (laf_rx_req_len % 1024)
		laf_rx_req_len = LAF_BULK_BUFFER_SIZE;

retry_rx_alloc:
	for (i = 0; i < laf_rx_reqs; i++) {
		req = laf_request_new(dev->ep_out, laf_rx_req_len);
		if (!req) {
			while ((req = laf_req_get(dev, &dev->rx_idle)))
				mtp_request_free(req, dev->ep_out);

			if (laf_tx_req_len <= LAF_BULK_BUFFER_SIZE) {
				pr_err("%s: rx_req_fail\n", __func__);
				goto rx_req_fail;
			}

			laf_rx_req_len = LAF_BULK_BUFFER_SIZE;
			laf_rx_reqs = LAF_RX_REQ_MAX;
			goto retry_rx_alloc;
		}
		req->length = laf_rx_req_len;
		req->complete = laf_complete_out;
		laf_req_put(dev, &dev->rx_idle, req);
	}

	return 0;

rx_req_fail:
	while ((req = laf_req_get(dev, &dev->tx_idle)))
		laf_request_free(req, dev->ep_in);
tx_req_fail:
	printk(KERN_ERR "laf_bind() could not allocate requests\n");
	return -1;
}

static ssize_t laf_read(struct file *fp, char __user *buf,
		size_t count, loff_t *pos)
{
	struct laf_dev *dev = fp->private_data;
#ifdef LAF_DEBUG
	struct usb_composite_dev *cdev = dev->cdev;
#endif
	struct usb_request *req;
	size_t r = count;
	unsigned xfer;
	int len;
	int ret = 0;

	LAF_DBG(cdev, "laf_read(%zu)\n", count);

	if (!dev->ep_out)
		return -ENODEV;

	len = ALIGN(count, dev->ep_out->maxpacket);

	if (len > laf_rx_req_len)
		return -EINVAL;

	/* we will block until we're online */
	LAF_DBG(cdev, "laf_read: waiting for online state\n");
	ret = wait_event_interruptible(dev->read_wq,
			dev->state != LAF_STATE_OFFLINE);
	if (req < 0) {
		r = ret;
		goto done;
	}
	spin_lock_irq(&dev->lock);
	dev->state |= LAF_STATE_READ;
	spin_unlock_irq(&dev->lock);

requeue_req:
	/* queue requests */
	LAF_VDBG(cdev, "queue requests\n");
	while ((req = laf_req_get(dev, &dev->rx_idle))) {
		if (usb_ep_queue(dev->ep_out, req, GFP_KERNEL) < 0) {
			pr_err("%s: usb_ep_queue fail\n", __func__);
			laf_req_put(dev, &dev->rx_idle, req);
			return -EIO;
		} else {
			LAF_VDBG(cdev, "rx %p queue\n", req);
		}
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible(dev->read_wq,
			((req = laf_req_get(dev, &dev->rx_pool))
			 || !(dev->state & LAF_STATE_READ)));
	if (ret < 0) {
		r = ret;
		goto done;
	}

	if (dev->state & LAF_STATE_READ) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0)
			goto requeue_req;

		LAF_DBG(cdev, "rx %p %d\n", req, req->actual);

		if (req->actual > count) {
			unsigned long flags;

			xfer = count;
			r = xfer;
			if (copy_to_user(buf, req->buf, xfer))
				r = -EFAULT;

			memmove(req->buf, req->buf + count, req->actual - count);

			spin_lock_irqsave(&dev->lock, flags);
			list_add(&req->list, &dev->rx_pool);
			spin_unlock_irqrestore(&dev->lock, flags);
		} else {
			xfer = (req->actual < count) ? req->actual : count;
			r = xfer;
			if (copy_to_user(buf, req->buf, xfer))
				r = -EFAULT;

			ret = usb_ep_queue(dev->ep_out, req, GFP_KERNEL);
			if (ret < 0) {
				pr_err("%s: usb_ep_queue fail\n", __func__);
				laf_req_put(dev, &dev->rx_idle, req);
				r = EIO;
			} else {
				LAF_DBG(cdev, "rx %p queue\n", req);
			}
		}
	} else
		r = -EIO;

done:
	spin_lock_irq(&dev->lock);
	if (dev->state != LAF_STATE_OFFLINE)
		dev->state &= ~LAF_STATE_READ;
	spin_unlock_irq(&dev->lock);

	LAF_DBG(cdev, "laf_read returning %zd\n", r);
	return r;
}

static ssize_t laf_write(struct file *fp, const char __user *buf,
		size_t count, loff_t *pos)
{
	struct laf_dev *dev = fp->private_data;
#ifdef LAF_DEBUG
	struct usb_composite_dev *cdev = dev->cdev;
#endif
	struct usb_request *req = 0;
	int r = count;
	unsigned xfer;
	int sendZLP = 0;
	int ret;

	LAF_DBG(cdev, "laf_write(%zu)\n", count);

	spin_lock_irq(&dev->lock);
	if (dev->state == LAF_STATE_OFFLINE) {
		spin_unlock_irq(&dev->lock);
		return -ENODEV;
	}
	dev->state |= LAF_STATE_WRITE;
	spin_unlock_irq(&dev->lock);

	/* we need to send a zero length packet to signal the end of transfer
	 * if the transfer size is aligned to a packet boundary.
	 */
	if ((count & (dev->ep_in->maxpacket - 1)) == 0)
		sendZLP = 1;

	while (count > 0 || sendZLP) {
		/* so we exit after sending ZLP */
		if (count == 0)
			sendZLP = 0;

		if (!(dev->state & LAF_STATE_WRITE)) {
			pr_debug("laf_write dev->error\n");
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
				((req = laf_req_get(dev, &dev->tx_idle))
				 || !(dev->state & LAF_STATE_WRITE)));

		if (ret < 0) {
			r = ret;
			break;
		}

		if (count > laf_tx_req_len)
			xfer = laf_tx_req_len;
		else
			xfer = count;
		if (xfer && copy_from_user(req->buf, buf, xfer)) {
			r = -EFAULT;
			break;
		}

		req->length = xfer;
		ret = usb_ep_queue(dev->ep_in, req, GFP_KERNEL);
		if (ret < 0) {
			LAF_DBG(cdev, "laf_write: xfer error %d\n", ret);
			r = -EIO;
			break;
		}

		buf += xfer;
		count -= xfer;

		/* zero this so we don't try to free it on error exit */
		req = 0;
	}

	if (req)
		laf_req_put(dev, &dev->tx_idle, req);

	spin_lock_irq(&dev->lock);
	if (dev->state != LAF_STATE_OFFLINE)
		dev->state &= ~LAF_STATE_WRITE;
	spin_unlock_irq(&dev->lock);

	LAF_DBG(cdev, "laf_write returning %d\n", r);
	return r;
}

static int laf_open(struct inode *ip, struct file *fp)
{
	pr_info("laf_open\n");
	if (laf_lock(&_laf_dev->open_excl))
		return -EBUSY;

	/* clear any error condition */
	if (_laf_dev->state != LAF_STATE_OFFLINE)
		_laf_dev->state = LAF_STATE_READY;

	fp->private_data = _laf_dev;

	return 0;
}

static int laf_release(struct inode *ip, struct file *fp)
{
	pr_info("laf_release\n");

	laf_unlock(&_laf_dev->open_excl);
	return 0;
}

/* file operations for /dev/laf */
static const struct file_operations laf_fops = {
	.owner   = THIS_MODULE,
	.read    = laf_read,
	.write   = laf_write,
	.open    = laf_open,
	.release = laf_release,
};

static struct miscdevice laf_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "laf",
	.fops  = &laf_fops,
};

	static int
laf_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct laf_dev *dev = func_to_laf(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
	LAF_DBG(cdev, "laf_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	laf_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = laf_create_bulk_endpoints(dev, &laf_fullspeed_in_desc,
			&laf_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		laf_highspeed_in_desc.bEndpointAddress =
			laf_fullspeed_in_desc.bEndpointAddress;
		laf_highspeed_out_desc.bEndpointAddress =
			laf_fullspeed_out_desc.bEndpointAddress;
	}
	/* support super speed hardware */
	if (gadget_is_superspeed(c->cdev->gadget)) {
		laf_superspeed_in_desc.bEndpointAddress =
			laf_fullspeed_in_desc.bEndpointAddress;
		laf_superspeed_out_desc.bEndpointAddress =
			laf_fullspeed_out_desc.bEndpointAddress;
	}

	LAF_DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
laf_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct laf_dev *dev = func_to_laf(f);
	struct usb_request *req;

	while ((req = laf_req_get(dev, &dev->tx_idle)))
		laf_request_free(req, dev->ep_in);
	while ((req = laf_req_get(dev, &dev->rx_idle)))
		laf_request_free(req, dev->ep_out);
	dev->state = LAF_STATE_OFFLINE;
}

static int laf_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct laf_dev *dev = func_to_laf(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	LAF_DBG(cdev, "laf_function_set_alt intf: %d alt: %d\n", intf, alt);

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret) {
		dev->ep_in->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
				dev->ep_in->name, ret);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_in);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
				dev->ep_in->name, ret);
		return ret;
	}

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret) {
		dev->ep_out->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
				dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_out);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
				dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	dev->state = LAF_STATE_READY;

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}

static void laf_function_disable(struct usb_function *f)
{
	struct laf_dev *dev = func_to_laf(f);
#ifdef LAF_DEBUG
	struct usb_composite_dev *cdev = dev->cdev;
#endif

	LAF_DBG(cdev, "laf_function_disable\n");
	dev->state = LAF_STATE_OFFLINE;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);

	LAF_VDBG(cdev, "%s disabled\n", dev->function.name);
}

static int laf_bind_config(struct usb_configuration *c)
{
	struct laf_dev *dev = _laf_dev;

	pr_info("laf_bind_config\n");

	dev->cdev = c->cdev;
	dev->function.name = "laf";
	dev->function.fs_descriptors = fs_laf_descs;
	dev->function.hs_descriptors = hs_laf_descs;
	if (gadget_is_superspeed(c->cdev->gadget))
		dev->function.ss_descriptors = ss_laf_descs;
	dev->function.bind = laf_function_bind;
	dev->function.unbind = laf_function_unbind;
	dev->function.set_alt = laf_function_set_alt;
	dev->function.disable = laf_function_disable;

	return usb_add_function(c, &dev->function);
}

static int laf_setup(void)
{
	struct laf_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spin_lock_init(&dev->lock);

	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);
	atomic_set(&dev->open_excl, 0);
	INIT_LIST_HEAD(&dev->tx_idle);
	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->rx_pool);

	_laf_dev = dev;

	ret = misc_register(&laf_device);
	if (ret)
		goto err;

	return 0;

err:
	_laf_dev = NULL;
	kfree(dev);
	printk(KERN_ERR "laf gadget driver failed to initialize\n");
	return ret;
}

static void laf_cleanup(void)
{
	struct laf_dev *dev = _laf_dev;

	if (!dev)
		return;

	misc_deregister(&laf_device);
	_laf_dev = NULL;
	kfree(dev);
}
