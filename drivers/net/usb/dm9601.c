/*
 * Davicom DM9620 USB 2.0 10/100Mbps ethernet devices
 *
 * Peter Korsgaard <jacmet@sunsite.dk>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 * V1.0 - ftp fail fixed
 * V1.1 - model name checking, & ether plug function enhancement [0x4f, 0x20]
 * V1.2 - init tx/rx checksum
 *      - fix dm_write_shared_word(), bug fix
 *      - fix 10 Mbps link at power saving mode fail  
 * V1.3 - Support kernel 2.6.31
 * V1.4 - Support eeprom write of ethtool 
 *        Support DM9685
 *        Transmit Check Sum Control by Optopn (Source Code Default: Disable)
 *        Recieve Drop Check Sum Error Packet Disable as chip default
 * V1.5 - Support RK2818 (Debug the Register Function)
 * V1.6 - Solve compiler issue for Linux 2.6.35
 * V1.7 - Enable MAC Layer Flow Control and define debug_message for linux version update.
 * V1.8 - Enable PHY Layer Flow Control, clear debug code, setup default phy_id value is 1.
 *        Update dm9620_mdio_read and dm9620_mdio_write.
 *        Fix bug of ethtool eeprom write       
 * V1.9 - Fixed "deverr" line 367 error in Linux 2.6.38
 * V2.0 - Fixed "dm9620_set_multicast" function CRC bug.
 * V2.1 - Add 802.3az for dm9621a
 * V2.2 - Add PID=0x1269 support CDC mode.
 * V2.3 - Add PID=0x0269 support CDC mode.       
 * V2.41 - Support Linux 3.6.9    
 * V2.42 - Work to V2.42 according to "DM9620 BulkOut ¸É¤B¤À¸Ñ.doc"
 * V2.43 - Special suport for DM9621A in the table 'products'
 * V2.45 - Fix the function TxStyle(), correct to be (len%2) from (len%1). 20131211.
 * V2.47 - Special suport for DM9621A by increase the table 'products', 20140617.
 * V2.49 - EEPROM_Utility_support (Report EEPROM_LEN from 256 to 128), 20140919
 * v2.49.1 - Debug dhcpcd (Update to V2.49.1a) 
 * v2.49.1f - Working around for DM9621A E1, pid/vid unstable
 */

//#define DEBUG
#define LNX_DM9620_VER_STR  "V2.49.1f"


#include <linux/module.h>
#include <linux/sched.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <linux/usb/usbnet.h>
#include <linux/ctype.h>
#include <linux/skbuff.h>   
#include <linux/version.h> // new v1.3

/* datasheet:
 http://www.davicom.com.tw
*/

/* control requests */
#define DM_READ_REGS	0x00
#define DM_WRITE_REGS	0x01
#define DM_READ_MEMS	0x02
#define DM_WRITE_REG	0x03
#define DM_WRITE_MEMS	0x05
#define DM_WRITE_MEM	0x07

/* registers */
#define DM_NET_CTRL	0x00
#define DM_RX_CTRL	0x05
#define DM_FLOW_CTRL	0x0a
#define DM_SHARED_CTRL	0x0b
#define DM_SHARED_ADDR	0x0c
#define DM_SHARED_DATA	0x0d	/* low + high */
#define DM_EE_PHY_L	0x0d
#define DM_EE_PHY_H	0x0e
#define DM_WAKEUP_CTRL  0x0f
#define DM_PHY_ADDR	0x10	/* 6 bytes */
#define DM_MCAST_ADDR	0x16	/* 8 bytes */
#define DM_GPR_CTRL	0x1e
#define DM_GPR_DATA	0x1f
#define DM_PID      0x2a
#define DM_XPHY_CTRL	0x2e
#define DM_TX_CRC_CTRL	0x31
#define DM_RX_CRC_CTRL	0x32 
#define DM_SMIREG       0x91
#define USB_CTRL	0xf4
#define PHY_SPEC_CFG	20
#define DM_TXRX_M       0x5C

#define DMSC_WEP	0x10
#define DMSC_ERPRW	0x02
#define DMSC_ERRE	0x01

#define MD96XX_EEPROM_MAGIC	0x9620
#define DM_MAX_MCAST	64
#define DM_MCAST_SIZE	8
#define DM_EEPROM_LEN	128
#define DM_TX_OVERHEAD	2	/* 2 byte header */
#define DM_RX_OVERHEAD_9601	7	/* 3 byte header + 4 byte crc tail */
#define DM_RX_OVERHEAD		8	/* 4 byte header + 4 byte crc tail */
#define DM_TIMEOUT	1000
#define DM_MODE9620     0x80
#define DM_TX_CS_EN	0        /* Transmit Check Sum Control */
#define DM9620_PHY_ID 1      /* Stone add For kernel read phy register */

struct dm96xx_priv {
  //int	flag_fail_count; // EVER RX-DBG
    int flg_txdbg; // NOW TX-DBG
	u8  mode_9620;	
	u8	tx_fix_mod;	
};     
#if defined(DEBUG)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,33)
#define dm9620_print(__dev, format, args...) netdev_dbg((__dev)->net, format, ##args) 
#define dm9620_err(__dev, format, args...) netdev_err((__dev)->net, format, ##args)
#else if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,33)
#define dm9620_print(dev, format, args...) devdbg(dev, format, ##args)
#define dm9620_err(dev, format, args...) deverr(dev, format, ##args)
#endif
#else
#define dm9620_print(dev, format, args...) printk(format, ##args)
#define dm9620_err(dev, format, args...) printk(format, ##args)
#endif
static int dm_read(struct usbnet *dev, u8 reg, u16 length, void *data)
{
//  	dm9620_print(dev, "dm_read() reg=0x%02x length=%d", reg, length);
	return usb_control_msg(dev->udev,
			       usb_rcvctrlpipe(dev->udev, 0),
			       DM_READ_REGS,
			       USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			       0, reg, data, length, USB_CTRL_SET_TIMEOUT); //USB_CTRL_SET_TIMEOUT V.S. USB_CTRL_GET_TIMEOUT
}

static int dm_read_reg(struct usbnet *dev, u8 reg, u8 *value)
{
	u16 *tmpwPtr;
	int ret;
	tmpwPtr= kmalloc (2, GFP_ATOMIC);
	if (!tmpwPtr)
	{
		printk("+++++++++++ JJ5 dm_read_reg() Error: can not kmalloc!\n"); //usbnet_suspend (intf, message);
		return 0; 
	}
	
	ret = dm_read(dev, reg, 2, tmpwPtr);  // usb_submit_urb v.s. usb_control_msg
	*value= (u8)(*tmpwPtr & 0xff);
	
	kfree (tmpwPtr);
	return ret;
}

static int dm_write(struct usbnet *dev, u8 reg, u16 length, void *data)
{
//  dm9620_print(dev, "dm_write() reg=0x%02x, length=%d", reg, length);
	return usb_control_msg(dev->udev,
			       usb_sndctrlpipe(dev->udev, 0),
			       DM_WRITE_REGS,
			       USB_DIR_OUT | USB_TYPE_VENDOR |USB_RECIP_DEVICE,
			       0, reg, data, length, USB_CTRL_SET_TIMEOUT);
}

static int dm_write_reg(struct usbnet *dev, u8 reg, u8 value)
{
//	dm9620_print(dev , "dm_write_reg() reg=0x%02x, value=0x%02x", reg, value);
	return usb_control_msg(dev->udev,
			       usb_sndctrlpipe(dev->udev, 0),
			       DM_WRITE_REG,
			       USB_DIR_OUT | USB_TYPE_VENDOR |USB_RECIP_DEVICE,
			       value, reg, NULL, 0, USB_CTRL_SET_TIMEOUT);
}

static void dm_write_async_callback(struct urb *urb)
{
	struct usb_ctrlrequest *req = (struct usb_ctrlrequest *)urb->context;

	if (urb->status < 0)
		printk(KERN_DEBUG "dm_write_async_callback() failed with %d\n",
		       urb->status);

	kfree(req);
	usb_free_urb(urb);
}

static void dm_write_async_helper(struct usbnet *dev, u8 reg, u8 value,
				  u16 length, void *data)
{
	struct usb_ctrlrequest *req;
	struct urb *urb;
	int status;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		dm9620_err(dev, "Error allocating URB in dm_write_async_helper!");
		return;
	}

	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_ATOMIC);
	if (!req) {
		dm9620_err(dev, "Failed to allocate memory for control request");
		usb_free_urb(urb);
		return;
	}

	req->bRequestType = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	req->bRequest = length ? DM_WRITE_REGS : DM_WRITE_REG;
	req->wValue = cpu_to_le16(value);
	req->wIndex = cpu_to_le16(reg);
	req->wLength = cpu_to_le16(length);

	usb_fill_control_urb(urb, dev->udev,
			     usb_sndctrlpipe(dev->udev, 0),
			     (void *)req, data, length,
			     dm_write_async_callback, req);

	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status < 0) {
		dm9620_err(dev, "Error submitting the control message: status=%d",
		       status);      
		kfree(req);
		usb_free_urb(urb);
	}
}

static void dm_write_async(struct usbnet *dev, u8 reg, u16 length, void *data)
{
//  dm9620_print(dev, "dm_write_async() reg=0x%02x length=%d", reg, length);
	dm_write_async_helper(dev, reg, 0, length, data);
}

static void dm_write_reg_async(struct usbnet *dev, u8 reg, u8 value)
{
//	dm9620_print(dev, "dm_write_reg_async() reg=0x%02x value=0x%02x",
//	       reg, value);      

	dm_write_async_helper(dev, reg, value, 0, NULL);
}

static int dm_read_shared_word(struct usbnet *dev, int phy, u8 reg, __le16 *value)
{
	int ret, i;
  u16 *tmpwPtr1;

	mutex_lock(&dev->phy_mutex);

	dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	dm_write_reg(dev, DM_SHARED_CTRL, phy ? 0xc : 0x4);

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp;

		udelay(1);
		ret = dm_read_reg(dev, DM_SHARED_CTRL, &tmp);
		if (ret < 0)
			goto out;

		/* ready */
		if ((tmp & 1) == 0)
			break;
	}

	if (i == DM_TIMEOUT) {
		dm9620_err(dev, "%s read timed out!", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	dm_write_reg(dev, DM_SHARED_CTRL, 0x0);
//	ret = dm_read(dev, DM_SHARED_DATA, 2, value); 
//Stone add
	tmpwPtr1= kmalloc (2, GFP_ATOMIC);
	if (!tmpwPtr1)
	{
		printk("+++++++++++ JJ5 dm_read_reg() Error: can not kmalloc!\n"); //usbnet_suspend (intf, message);
		return 0; 
	}
	
	ret = dm_read(dev, DM_SHARED_DATA, 2, tmpwPtr1);  // usb_submit_urb v.s. usb_control_msg
	*value= (u16)(*tmpwPtr1 & 0xffff);
	
	kfree (tmpwPtr1); 

//	dm9620_print(dev, "read shared %d 0x%02x returned 0x%04x, %d",
//	       phy, reg, *value, ret);      

 out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static int dm_write_shared_word(struct usbnet *dev, int phy, u8 reg, __le16 value)
{
	int ret, i;

	mutex_lock(&dev->phy_mutex);

	ret = dm_write(dev, DM_SHARED_DATA, 2, &value);
	if (ret < 0)
		goto out;

	dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	if (!phy) dm_write_reg(dev, DM_SHARED_CTRL, 0x10);
	dm_write_reg(dev, DM_SHARED_CTRL, phy ? 0x0a : 0x12);
	dm_write_reg(dev, DM_SHARED_CTRL, 0x10);

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp;

		udelay(1);
		ret = dm_read_reg(dev, DM_SHARED_CTRL, &tmp);
		if (ret < 0)
			goto out;

		/* ready */
		if ((tmp & 1) == 0)
			break;
	}

	if (i == DM_TIMEOUT) {
		dm9620_err(dev,"%s write timed out!", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	dm_write_reg(dev, DM_SHARED_CTRL, 0x0);

out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static void device_polling(struct usbnet *dev, u8 reg, u8 dmsc_bit, 
          u8 uexpected)
{
	int i,ret;
	u8 tmp= 0;
	for (i = 0; i < DM_TIMEOUT; i++) {
		udelay(1);
		ret = dm_read_reg(dev, reg, &tmp);
		if (ret < 0){
		        dm9620_err(dev,
				"[dm962 read reg] (reg: 0x%02x) error!\n", 
                reg);
			break;
        }
		if ((tmp & dmsc_bit) == uexpected) /* ready */
			break;
	}
	if (i == DM_TIMEOUT)
		dm9620_err(dev, "[dm962 time out] on polling bit:0x%x\n",
          dmsc_bit);
}

static void dm_write_eeprom_word(struct usbnet *dev, u8 offset, u8 *data)
{
    //offset= offset / 2;  /*dm9620_write_eeprom(dev, offset / 2, data);*/
	mutex_lock(&dev->phy_mutex);

    dm_write_reg(dev, DM_SHARED_ADDR, offset);
	dm_write_reg(dev, DM_EE_PHY_H, data[1]);
	dm_write_reg(dev, DM_EE_PHY_L, data[0]);
    dm_write_reg(dev, DM_SHARED_CTRL, DMSC_WEP | DMSC_ERPRW);
        device_polling(dev, DM_SHARED_CTRL, DMSC_ERRE, 0x00);			
	dm_write_reg(dev, DM_SHARED_CTRL, 0);

	mutex_unlock(&dev->phy_mutex);
}

static int dm_read_eeprom_word(struct usbnet *dev, u8 offset, void *value)
{
	return dm_read_shared_word(dev, 0, offset, value);
}

static int dm9620_set_eeprom(struct net_device *net,
        struct ethtool_eeprom *eeprom, u8 *data)
{
	struct usbnet *dev = netdev_priv(net);
	int offset = eeprom->offset;
	int len = eeprom->len;
	int done;
	
//	dm9620_print(dev, "EEPROM: magic value, magic = 0x%x offset =0x%x data = 0x%x ",eeprom->magic, eeprom->offset,*data);
	if (eeprom->magic != MD96XX_EEPROM_MAGIC) {
		dm9620_print(dev, "EEPROM: magic value mismatch, magic = 0x%x",
			eeprom->magic);	
		return -EINVAL;
	}

	while (len > 0) {
		if (len & 1 || offset & 1) {
			int which = offset & 1;
			u8 tmp[2];
            dm_read_eeprom_word(dev, offset / 2, tmp);
			tmp[which] = *data;
            dm_write_eeprom_word(dev, offset / 2, tmp); 
			mdelay(10);
			done = 1;
		} else {
	dm_write_eeprom_word(dev, offset / 2, data);  
			done = 2;
		}
		data += done;
		offset += done;
		len -= done;
	}
	return 0;
}

static int dm9620_get_eeprom_len(struct net_device *dev)
{
	return DM_EEPROM_LEN;
}

static int dm9620_get_eeprom(struct net_device *net,
			     struct ethtool_eeprom *eeprom, u8 * data)
{
	struct usbnet *dev = netdev_priv(net);
	__le16 *ebuf = (__le16 *) data;
	int i;

	/* access is 16bit */
	if ((eeprom->offset % 2) || (eeprom->len % 2))
		return -EINVAL;

	for (i = 0; i < eeprom->len / 2; i++) {
		if (dm_read_eeprom_word(dev, eeprom->offset / 2 + i,
					&ebuf[i]) < 0)
			return -EINVAL;
	}
	return 0;
}

static int dm9620_mdio_read(struct net_device *netdev, int phy_id, int loc)
{
	struct usbnet *dev = netdev_priv(netdev);

	__le16 res;

	dm_read_shared_word(dev, phy_id, loc, &res);

//  dm9620_print(dev, "dm9620_mdio_read() phy_id=0x%02x, loc=0x%02x, returns=0x%04x",
//	       phy_id, loc, le16_to_cpu(res));
	return le16_to_cpu(res);
}

static void dm9620_mdio_write(struct net_device *netdev, int phy_id, int loc,
			      int val)
{
	struct usbnet *dev = netdev_priv(netdev);
	__le16 res = cpu_to_le16(val);
	int mdio_val;

//	dm9620_print(dev, "dm9620_mdio_write() phy_id=0x%02x, loc=0x%02x, val=0x%04x",
//	       phy_id, loc, val);      

	dm_write_shared_word(dev, phy_id, loc, res);
	mdelay(1);
	mdio_val = dm9620_mdio_read(netdev, phy_id, loc);

}

static void dm9620_get_drvinfo(struct net_device *net,
			       struct ethtool_drvinfo *info)
{
	/* Inherit standard device info */
	usbnet_get_drvinfo(net, info);
	info->eedump_len = DM_EEPROM_LEN;
}

static u32 dm9620_get_link(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	return mii_link_ok(&dev->mii);
}

static int dm9620_ioctl(struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}


#define DM_LINKEN  (1<<5)
#define DM_MAGICEN (1<<3)
#define DM_LINKST  (1<<2)
#define DM_MAGICST (1<<0)

static void
dm9620_get_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	u8 opt;

	if (dm_read_reg(dev, DM_WAKEUP_CTRL, &opt) < 0) {
		wolinfo->supported = 0;
		wolinfo->wolopts = 0;
		return;
	}
	wolinfo->supported = WAKE_PHY | WAKE_MAGIC;
	wolinfo->wolopts = 0;

	if (opt & DM_LINKEN)
		wolinfo->wolopts |= WAKE_PHY;
	if (opt & DM_MAGICEN)
		wolinfo->wolopts |= WAKE_MAGIC;
}


static int
dm9620_set_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	u8 opt = 0;

	if (wolinfo->wolopts & WAKE_PHY)
		opt |= DM_LINKEN;
	if (wolinfo->wolopts & WAKE_MAGIC)
		opt |= DM_MAGICEN;

	dm_write_reg(dev, DM_NET_CTRL, 0x48);  // enable WAKEEN 
	
//	dm_write_reg(dev, 0x92, 0x3f); //keep clock on Hank Jun 30
	
	return dm_write_reg(dev, DM_WAKEUP_CTRL, opt);
}

static struct ethtool_ops dm9620_ethtool_ops = {
	.get_drvinfo	= dm9620_get_drvinfo,
	.get_link	= dm9620_get_link,
	.get_msglevel	= usbnet_get_msglevel,
	.set_msglevel	= usbnet_set_msglevel,
	.get_eeprom_len	= dm9620_get_eeprom_len,
	.get_eeprom	= dm9620_get_eeprom,
	.set_eeprom	= dm9620_set_eeprom,
	.get_settings	= usbnet_get_settings,
	.set_settings	= usbnet_set_settings,
	.nway_reset	= usbnet_nway_reset,
	.get_wol	= dm9620_get_wol,
	.set_wol	= dm9620_set_wol,
};

static void dm9620_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	/* We use the 20 byte dev->data for our 8 byte filter buffer
	 * to avoid allocating memory that is tricky to free later */
	u8 *hashes = (u8 *) & dev->data;
	u8 rx_ctl = 0x31;

	memset(hashes, 0x00, DM_MCAST_SIZE);
	hashes[DM_MCAST_SIZE - 1] |= 0x80;	/* broadcast address */

	if (net->flags & IFF_PROMISC) {
		rx_ctl |= 0x02;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,33)	
	} else if (net->flags & IFF_ALLMULTI ||  netdev_mc_count(net) > DM_MAX_MCAST) {
		rx_ctl |= 0x8;
	} else if (!netdev_mc_empty(net)) {
            struct netdev_hw_addr *ha;
 
         netdev_for_each_mc_addr(ha, net) {
              u32 crc = crc32_le(~0, ha->addr, ETH_ALEN) & 0x3f;
              hashes[crc>>3] |= 1 << (crc & 0x7);
		}
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,33)
  	} else if (net->flags & IFF_ALLMULTI || net->mc_count > DM_MAX_MCAST) {
		rx_ctl |= 0x08;
	} else if (net->mc_count) {
		struct dev_mc_list *mc_list = net->mc_list;
		int i;

		for (i = 0; i < net->mc_count; i++, mc_list = mc_list->next) {
			u32 crc = crc32_le(~0, mc_list->dmi_addr, ETH_ALEN) & 0x3f;
                        hashes[crc>>3] |= 1 << (crc & 0x7);
		} 
#endif		
	}
 
	dm_write_async(dev, DM_MCAST_ADDR, DM_MCAST_SIZE, hashes);
	dm_write_reg_async(dev, DM_RX_CTRL, rx_ctl);
}

 
 static void __dm9620_set_mac_address(struct usbnet *dev)
 {
         dm_write_async(dev, DM_PHY_ADDR, ETH_ALEN, dev->net->dev_addr);
 }
 
 static int dm9620_set_mac_address(struct net_device *net, void *p)
 {
         struct sockaddr *addr = p;
         struct usbnet *dev = netdev_priv(net);
	 int i;
 
#if 1
	 printk("[dm96] Set mac addr %pM\n", addr->sa_data);  // %x:%x:...
	 printk("[dm96] ");
	 for (i=0; i<net->addr_len; i++)
	 printk("[%02x] ", addr->sa_data[i]);
	 printk("\n");
 #endif
 
         if (!is_valid_ether_addr(addr->sa_data)) {
                 dev_err(&net->dev, "not setting invalid mac address %pM\n",
                                                                 addr->sa_data);
                 return -EINVAL;
         }
 
         memcpy(net->dev_addr, addr->sa_data, net->addr_len);
         __dm9620_set_mac_address(dev);
 
         return 0;
 }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 

static const struct net_device_ops vm_netdev_ops= { // new kernel 2.6.31  (20091217JJ)

            .ndo_open               = usbnet_open,  
            .ndo_stop               = usbnet_stop,  
            .ndo_start_xmit         = usbnet_start_xmit, 
            .ndo_tx_timeout         = usbnet_tx_timeout, 
            .ndo_change_mtu         = usbnet_change_mtu, 
            .ndo_validate_addr      = eth_validate_addr, 
	    .ndo_do_ioctl	    = dm9620_ioctl,   
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
            .ndo_set_rx_mode        = dm9620_set_multicast,   
#else
	    .ndo_set_multicast_list = dm9620_set_multicast,   
#endif
            .ndo_set_mac_address    = dm9620_set_mac_address,  
};
#endif

static int dm9620_bind(struct usbnet *dev, struct usb_interface *intf)
{
  u16 *tmpwPtr2;
	int ret,mdio_val,i;
	struct dm96xx_priv* priv;
	u8 temp;
	u8 tmp;

	ret = usbnet_get_endpoints(dev, intf);
	if (ret)
		goto out;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 
	dev->net->netdev_ops = &vm_netdev_ops; // new kernel 2.6.31  (20091217JJ)
	dev->net->ethtool_ops = &dm9620_ethtool_ops;
#else
	dev->net->do_ioctl = dm9620_ioctl;  
	dev->net->set_multicast_list = dm9620_set_multicast;
	dev->net->ethtool_ops = &dm9620_ethtool_ops;
#endif
	dev->net->hard_header_len += DM_TX_OVERHEAD;
	dev->hard_mtu = dev->net->mtu + dev->net->hard_header_len;
	dev->rx_urb_size = dev->net->mtu + ETH_HLEN + DM_RX_OVERHEAD+1; // ftp fail fixed

	dev->mii.dev = dev->net;
	dev->mii.mdio_read = dm9620_mdio_read;
	dev->mii.mdio_write = dm9620_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0x1f;
	dev->mii.phy_id = DM9620_PHY_ID;

	printk("[dm962] Linux Driver = %s\n", LNX_DM9620_VER_STR);
//JJ1
	if ( (ret= dm_read_reg(dev, 0x29, &tmp)) >=0)
		printk("++++++[dm962]+++++ dm_read_reg() 0x29 0x%02x\n",tmp);
	else
		printk("++++++[dm962]+++++ dm_read_reg() 0x29 fail-func-return %d\n", ret);
		
	if ( (ret= dm_read_reg(dev, 0x28, &tmp)) >=0)
		printk("++++++[dm962]+++++ dm_read_reg() 0x28 0x%02x\n",tmp);
	else
		printk("++++++[dm962]+++++ dm_read_reg() 0x28 fail-func-return %d\n", ret);
		
	if ( (ret= dm_read_reg(dev, 0x2b, &tmp)) >=0)
		printk("++++++[dm962]+++++ dm_read_reg() 0x2b 0x%02x\n",tmp);
	else
		printk("++++++[dm962]+++++ dm_read_reg() 0x2b fail-func-return %d\n", ret);
	if ( (ret= dm_read_reg(dev, 0x2a, &tmp)) >=0)
		printk("++++++[dm962]+++++ dm_read_reg() 0x2a 0x%02x\n",tmp);
	else
		printk("++++++[dm962]+++++ dm_read_reg() 0x2a fail-func-return %d\n", ret);
		
//JJ3
	if ( (ret= dm_read_reg(dev, 0xF2, &tmp)) >=0)
		printk("++++++[dm962]+++++ dm_read_reg() 0xF2 0x%02x\n",tmp);
	else
		printk("++++++[dm962]+++++ dm_read_reg() 0xF2 fail-func-return %d\n", ret);
		
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[7] %d %s\n", tmp>>7, (tmp&(1<<7))? "Err: RX Unexpected condition": "OK" );
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[6] %d %s\n", (tmp>>6)&1, (tmp&(1<<6))? "Err: Host Suspend condition": "OK" );
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[5] %d %s\n", (tmp>>5)&1, (tmp&(1<<5))? "EP1: Data Ready": "EP1: Empty" );
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[3] %d %s\n", (tmp>>3)&1, (tmp&(1<<3))? "Err: Bulk out condition": "OK" );
		
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[2] %d %s\n", (tmp>>2)&1, (tmp&(1<<2))? "Err: TX Buffer full": "OK" );
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[1] %d %s\n", (tmp>>1)&1, (tmp&(1<<1))? "Warn: TX buffer Almost full": "OK" );
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[0] %d %s\n", (tmp>>0)&1, (tmp&(1<<0))? "Status: TX buffer has pkts": "Status: TX buffer 0 pkts" );

	/* reset */
	dm_write_reg(dev, DM_NET_CTRL, 1);
	udelay(20);
	//Stone add Enable "MAC layer" Flow Control, TX Pause Packet Enable and 
	dm_write_reg(dev, DM_FLOW_CTRL, 0x29);
	//Stone add Enable "PHY layer" Flow Control support (phy register 0x04 bit 10)
	temp = dm9620_mdio_read(dev->net, dev->mii.phy_id, 0x04);
	dm9620_mdio_write(dev->net, dev->mii.phy_id, 0x04, temp | 0x400);
	

	/* Add V1.1, Enable auto link while plug in RJ45, Hank July 20, 2009*/
	dm_write_reg(dev, USB_CTRL, 0x20); 
	/* read MAC */
	if (dm_read(dev, DM_PHY_ADDR, ETH_ALEN, dev->net->dev_addr) < 0) {
		printk(KERN_ERR "Error reading MAC address\n");
		ret = -ENODEV;
		goto out;
	}

#if 1
	 printk("[dm96] Chk mac addr %pM\n", dev->net->dev_addr);  // %x:%x...
	 printk("[dm96] ");
	 for (i=0; i<ETH_ALEN; i++)
	 printk("[%02x] ", dev->net->dev_addr[i]);
	 printk("\n");
#endif

	/* read SMI mode register */
        priv = dev->driver_priv = kmalloc(sizeof(struct dm96xx_priv), GFP_ATOMIC);
	if (!priv) {
		dm9620_err(dev,"Failed to allocate memory for dm96xx_priv");
		ret = -ENOMEM;
		goto out;
	}
	
        /* work-around for 9620 mode */
	dm_read_reg(dev, 0x5c, &temp); 
	priv->tx_fix_mod = temp;
	printk(KERN_WARNING "[dm96] 9620 tx_fix_mod (DM9_NREV= %d)\n", priv->tx_fix_mod);

	printk("[dm96] Fixme: work around for 9620 mode\n");
	printk("[dm96] Add tx_fixup() debug...\n");
	dm_write_reg(dev, DM_MCAST_ADDR, 0);     // clear data bus to 0s
	dm_read_reg(dev, DM_MCAST_ADDR, &temp);  // clear data bus to 0s
	ret = dm_read_reg(dev, DM_SMIREG, &temp);   // Must clear data bus before we can read the 'MODE9620' bit

	priv->flg_txdbg= 0; //->flag_fail_count= 0;
	if (ret<0) {
		printk(KERN_ERR "[dm96] Error read SMI register\n");
	}
	else priv->mode_9620 = temp & DM_MODE9620;

	printk(KERN_WARNING "[dm96] 9620 Mode = %d\n", priv->mode_9620);
	
	dm_read_reg(dev, DM_TXRX_M, &temp);  // Need to check the Chipset version (register 0x5c is 0x02?)
	if (temp == 0x02)
	{
	 dm_read_reg(dev, 0x3f, &temp);
	 temp |= 0x80; 
   dm_write_reg(dev, 0x3f, temp);
   }
  
  //Stone add for check Product ID == 0x1269
  tmpwPtr2= kmalloc (2, GFP_ATOMIC);
	if (!tmpwPtr2)
	{
		printk("+++++++++++ JJ5 dm_read_reg() Error: can not kmalloc!\n"); //usbnet_suspend (intf, message);
		return 0; 
	} 
  ret =dm_read(dev, DM_PID, 2, tmpwPtr2);

  if (*tmpwPtr2 == 0x1269)
   dm_write_reg(dev, DM_SMIREG, 0xa0);
   
  if (*tmpwPtr2 == 0x0269)
   dm_write_reg(dev, DM_SMIREG, 0xa0); 
  
  kfree (tmpwPtr2); 
	
	/* power up phy */
	dm_write_reg(dev, DM_GPR_CTRL, 1);
	dm_write_reg(dev, DM_GPR_DATA, 0);

	/* Init tx/rx checksum */
#if 	DM_TX_CS_EN
	dm_write_reg(dev, DM_TX_CRC_CTRL, 7);
#endif 
	dm_write_reg(dev, DM_RX_CRC_CTRL, 2);

	/* receive broadcast packets */
	dm9620_set_multicast(dev->net);
	dm9620_mdio_write(dev->net, dev->mii.phy_id, MII_BMCR, BMCR_RESET);

	/* Hank add, work for comapubility issue (10M Power control) */	
	
	dm9620_mdio_write(dev->net, dev->mii.phy_id, PHY_SPEC_CFG, 0x800);
	mdio_val = dm9620_mdio_read(dev->net, dev->mii.phy_id, PHY_SPEC_CFG);
	
	dm9620_mdio_write(dev->net, dev->mii.phy_id, MII_ADVERTISE,
			  ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
	mii_nway_restart(&dev->mii); 
	
out:
	return ret;
}

void dm9620_unbind(struct usbnet *dev, struct usb_interface *intf)
{
struct dm96xx_priv* priv= dev->driver_priv;
	printk("dm9620_unbind():\n");

   //printk("flag_fail_count  %lu\n", (long unsigned int)priv->flag_fail_count);
	printk("flg_txdbg  %lu\n", (long unsigned int)priv->flg_txdbg);
	kfree(dev->driver_priv); // displayed dev->.. above, then can free dev 

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 
	printk("rx_length_errors %lu\n",dev->net->stats.rx_length_errors);
	printk("rx_over_errors   %lu\n",dev->net->stats.rx_over_errors  );
	printk("rx_crc_errors    %lu\n",dev->net->stats.rx_crc_errors   );
	printk("rx_frame_errors  %lu\n",dev->net->stats.rx_frame_errors );
	printk("rx_fifo_errors   %lu\n",dev->net->stats.rx_fifo_errors  );
	printk("rx_missed_errors %lu\n",dev->net->stats.rx_missed_errors);	
#else
	printk("rx_length_errors %lu\n",dev->stats.rx_length_errors);
	printk("rx_over_errors   %lu\n",dev->stats.rx_over_errors  );
	printk("rx_crc_errors    %lu\n",dev->stats.rx_crc_errors   );
	printk("rx_frame_errors  %lu\n",dev->stats.rx_frame_errors );
	printk("rx_fifo_errors   %lu\n",dev->stats.rx_fifo_errors  );
	printk("rx_missed_errors %lu\n",dev->stats.rx_missed_errors);	
#endif


}

struct dhcp_msg {
  u8 op, htype, hlen, hops;
  u8 xid[4];
  u16 secs, flags;
  u8 ciaddr[4];
  u8 yiaddr[4];
  u8 siaddr[4];
  u8 giaddr[4];
  u8 chaddr[16];
#if 1 
  // ~DHCP_LIGHT with u8 [64] + u8 [128]
  u8 sname[64];
  u8 file[128];
#endif
  u8 options[312];
};

#define BUF ((struct dhcp_msg *)&skb->data[4+42])
#define TXBUF ((struct dhcp_msg *)&skb->data[2+42])

static int dm9620_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	u8 status;
	int len;
    int i;
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;

	/* 9620 format:
	   b0: rx status
	   b1: packet length (incl crc) low
	   b2: packet length (incl crc) high
	   b3..n-4: packet data
	   bn-3..bn: ethernet crc
	 */

	/* 9620 format:
	   one additional byte then 9620 : 
	   rx_flag in the first pos
	 */

	if (unlikely(skb->len < DM_RX_OVERHEAD_9601)) {   // 20090623
		dev_err(&dev->udev->dev, "unexpected tiny rx frame\n");
		return 0;
	}

	if (priv->mode_9620) {
		/* mode 9620 */

		if (unlikely(skb->len < DM_RX_OVERHEAD)) {  // 20090623
			dev_err(&dev->udev->dev, "unexpected tiny rx frame\n");
			return 0;
		}
		
		//	if (skb->data[0]!=0x01)
		//		priv->flag_fail_count++;
	
		status = skb->data[1];
		len = (skb->data[2] | (skb->data[3] << 8)) - 4;
		
		if (unlikely(status & 0xbf)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 
			if (status & 0x01) dev->net->stats.rx_fifo_errors++;
			if (status & 0x02) dev->net->stats.rx_crc_errors++;
			if (status & 0x04) dev->net->stats.rx_frame_errors++;
			if (status & 0x20) dev->net->stats.rx_missed_errors++;
			if (status & 0x90) dev->net->stats.rx_length_errors++;
#else
			if (status & 0x01) dev->stats.rx_fifo_errors++;
			if (status & 0x02) dev->stats.rx_crc_errors++;
			if (status & 0x04) dev->stats.rx_frame_errors++;
			if (status & 0x20) dev->stats.rx_missed_errors++;
			if (status & 0x90) dev->stats.rx_length_errors++;
#endif
			return 0;
		}

//Bootstrap Protocol
if (BUF->op==0x02 && BUF->htype==0x01)
{
  //Get 'DHCP offer'
  //Get 'DHCP ACK'
  for (i= 0; i<96; i++) // a suitable lenght
  {
    if (BUF->options[i]==0x35 &&  BUF->options[i+1]==0x01)
    {
       if (BUF->options[i+2]==0x02)
       {
         // DHCP Offer
         printk("[dm96-get-DHCP Offer] %d.%d.%d.%d\n",
             BUF->yiaddr[0], BUF->yiaddr[1], BUF->yiaddr[2], BUF->yiaddr[3]);
         printk("[dm96-get-DHCP Offer] %02x:%02x:%02x: %02x:%02x:%02x\n",
             BUF->chaddr[0], BUF->chaddr[1], BUF->chaddr[2], 
             BUF->chaddr[3], BUF->chaddr[4], BUF->chaddr[5]);
       }
       else if (BUF->options[i+2]==0x05)
       {
         // DHCP ACK
         printk("[dm96-get-DHCP ACK] %d.%d.%d.%d\n",
             BUF->yiaddr[0], BUF->yiaddr[1], BUF->yiaddr[2], BUF->yiaddr[3]);
         printk("[dm96-get-DHCP ACK] %02x:%02x:%02x: %02x:%02x:%02x\n",
             BUF->chaddr[0], BUF->chaddr[1], BUF->chaddr[2], 
             BUF->chaddr[3], BUF->chaddr[4], BUF->chaddr[5]);
       }
       break;
    }
  }
}

		skb_pull(skb, 4);
		skb_trim(skb, len);

	}
	else { /* mode 9620 (original driver code) */
		status = skb->data[0];
		len = (skb->data[1] | (skb->data[2] << 8)) - 4;
		
		if (unlikely(status & 0xbf)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 
			if (status & 0x01) dev->net->stats.rx_fifo_errors++;
			if (status & 0x02) dev->net->stats.rx_crc_errors++;
			if (status & 0x04) dev->net->stats.rx_frame_errors++;
			if (status & 0x20) dev->net->stats.rx_missed_errors++;
			if (status & 0x90) dev->net->stats.rx_length_errors++;
#else
			if (status & 0x01) dev->stats.rx_fifo_errors++;
			if (status & 0x02) dev->stats.rx_crc_errors++;
			if (status & 0x04) dev->stats.rx_frame_errors++;
			if (status & 0x20) dev->stats.rx_missed_errors++;
			if (status & 0x90) dev->stats.rx_length_errors++;
#endif
			return 0;
		}

		skb_pull(skb, 3);
		skb_trim(skb, len);
	}

	return 1;
} // 'priv'

#define TX_LEN_E  (1<<0)  //EVEN, No action
#define TX_LEN_O  (1<<1)  //ODD, Odd to even workaround
#define TX_LEN_F  (1<<2)  //FULL, Full payload workaround
u8 TxStyle(int len, unsigned full_payload){
  u8 s= (len%2)? TX_LEN_O: TX_LEN_E;
  len= ((len+1)/2)*2;
  len += 2;
  if ((len % full_payload)==0)
    s |= TX_LEN_F;
  return s;
}
struct sk_buff *TxExpend(struct dm96xx_priv* priv, u8 ts, struct sk_buff *skb, gfp_t flags)
{
    int newheadroom= 2, newtailroom= 0;  
    if (ts&TX_LEN_O) newtailroom++;
    if (ts&TX_LEN_F) newtailroom += 2;
    if (skb_headroom(skb) >= newheadroom) newheadroom= 0; // head no need expend
    if (skb_tailroom(skb) >= newtailroom) newtailroom= 0; // tail no need expend
    if (newheadroom || newtailroom){
		struct sk_buff *skb2;
		skb2 = skb_copy_expand(skb, newheadroom, newtailroom, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb){
			printk("[dm96-TxRound].%d expend copy fail, for head, tail= %d, %d\n", priv->flg_txdbg++, newheadroom, newtailroom);
			return NULL;
		}
		printk("[dm96-TxRound].%d expend copy OK, for head, tail= %d, %d\n", priv->flg_txdbg++, newheadroom, newtailroom);
    }
    return skb;
}
static struct sk_buff *dm9620_tx_fixup(struct usbnet *dev, struct sk_buff *skb,
				       gfp_t flags)
{
	int len;
    int newheadroom, newtailroom;  
    int i, j;
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;

	/* format:
	   b0: packet length low
	   b1: packet length high
	   b3..n: packet data
	*/

	len = skb->len;

  if (priv->tx_fix_mod<3)
  {
    /*
	if (skb_headroom(skb) < DM_TX_OVERHEAD) {
		struct sk_buff *skb2;
		skb2 = skb_copy_expand(skb, DM_TX_OVERHEAD, 0, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

	__skb_push(skb, DM_TX_OVERHEAD);

	if ((skb->len % dev->maxpacket) == 0)
		len++;
    */
    //;DM9620-E4,E5, and E6
	/* usbnet adds padding 1 byte if odd len */
	/* usbnet adds padding 2 bytes if length is a multiple of packet size
	   if so, adjust length value in header */
     u8 TS= TxStyle(len, dev->maxpacket); //
     if (!(skb= TxExpend(priv, TS, skb, flags))) return NULL; //

     if (TS & TX_LEN_F) len += 2;

     newheadroom= 2; //2
     newtailroom= 0; //0, 1, 2, or 3
     if (TS & TX_LEN_O) newtailroom++;
     if (TS & TX_LEN_F) newtailroom += 2;
     
   //if (TS & TX_LEN_O) printk("[dm96-TxRound].%d for LEN_ODD tail_room +1, rslt add %d\n", priv->flg_txdbg, newtailroom);
   //if (TS & TX_LEN_F) printk("[dm96-TxRound].%d for LEN_PLOAD tail_room +2, rslt add %d\n", priv->flg_txdbg, newtailroom);
   //if (TS & TX_LEN_F) printk("[dm96-TxRound].%d for LEN_PLOAD data_len +2, len from %d to %d\n", priv->flg_txdbg, len-2, len);
     if (TS & (TX_LEN_O|TX_LEN_F)) priv->flg_txdbg++;

	__skb_push(skb, newheadroom); //2 bytes,for data[0],data[1]
    __skb_put(skb, newtailroom); //0, 1, 2, or 3 bytes (for tailer), 
                                 //Note: 0, NOTHING
                                 //Note: 1, Odd to even WORKAROUND.
                                 //Note: 2 or 3, the condition is full payload,
                                 // This is the add more two bytes WORKAROUND
                                 // for bulkout and buffLen.
  }
  else 
  {
    //;DM9620-E7
	if (skb_headroom(skb) < DM_TX_OVERHEAD) {
		struct sk_buff *skb2;
		skb2 = skb_copy_expand(skb, DM_TX_OVERHEAD, 0, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

   newheadroom= 2; //2
	__skb_push(skb, newheadroom);  //2 bytes, for data[0],data[1]
  }


	skb->data[0] = len;
	skb->data[1] = len >> 8;

//Bootstrap Protocol
if (TXBUF->op==0x01 && TXBUF->htype==0x01)
{
  //Get 'DHCP Discover'
  //Get 'DHCP Request'
  for (i= 0, j= 0; (i<96) && (j<2); i++) // a suitable lenght
  {
    if (TXBUF->options[i]==0x35 &&  TXBUF->options[i+1]==0x01)
    {
       if (TXBUF->options[i+2]==0x01)
       {
         // DHCP Discover
         printk("[dm96-set-DHCP Discover] %02x:%02x:%02x: %02x:%02x:%02x\n",
             TXBUF->chaddr[0], TXBUF->chaddr[1], TXBUF->chaddr[2], 
             TXBUF->chaddr[3], TXBUF->chaddr[4], TXBUF->chaddr[5]);
         printk("[dm96-not-has-DHCP IP] %d.%d.%d.%d\n",
             TXBUF->yiaddr[0], TXBUF->yiaddr[1], TXBUF->yiaddr[2], TXBUF->yiaddr[3]);
         j++;
       }
       else if (TXBUF->options[i+2]==0x03)
       {
         // DHCP Request
         printk("[dm96-set-DHCP Request] %02x:%02x:%02x: %02x:%02x:%02x\n",
             TXBUF->chaddr[0], TXBUF->chaddr[1], TXBUF->chaddr[2], 
             TXBUF->chaddr[3], TXBUF->chaddr[4], TXBUF->chaddr[5]);
       }
       i += 2; //break;
       j++;
    }

    if (TXBUF->options[i]==0x32 &&  TXBUF->options[i+1]==0x04)
    {
         // DHCP Request
         printk("[dm96-set-DHCP Request] %d.%d.%d.%d\n",
             TXBUF->options[i+2], TXBUF->options[i+3], TXBUF->options[i+4], TXBUF->options[i+5]);
         i += 5; //break;
         j++;
    }
  } //.for
} //.if


	/* hank, recalcute checksum of TCP */

	
	return skb;
} // 'kb'

static void dm9620_status(struct usbnet *dev, struct urb *urb)
{
	int link;
	u8 *buf;

	/* format:
	   b0: net status
	   b1: tx status 1
	   b2: tx status 2
	   b3: rx status
	   b4: rx overflow
	   b5: rx count
	   b6: tx count
	   b7: gpr
	*/

	if (urb->actual_length < 8)
		return;

	buf = urb->transfer_buffer;

	link = !!(buf[0] & 0x40);
	if (netif_carrier_ok(dev->net) != link) {
		if (link) {
			netif_carrier_on(dev->net);
			usbnet_defer_kevent (dev, EVENT_LINK_RESET);
		}
		else
			netif_carrier_off(dev->net);
		dm9620_print(dev, "Link Status is: %d", link);
	}
}

static int dm9620_link_reset(struct usbnet *dev)
{
	struct ethtool_cmd ecmd;
	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);
	/* hank add*/
dm9620_mdio_write(dev->net, dev->mii.phy_id, PHY_SPEC_CFG, 0x800);
	dm9620_print(dev, "link_reset() speed: %d duplex: %d",
	       ecmd.speed, ecmd.duplex);      
	
	return 0;
}

static const struct driver_info dm9620_info = {
	.description	= "Davicom DM9620 USB Ethernet",
	.flags		= FLAG_ETHER,
	.bind		= dm9620_bind,
	.rx_fixup	= dm9620_rx_fixup,
	.tx_fixup	= dm9620_tx_fixup,
	.status		= dm9620_status,
	.link_reset	= dm9620_link_reset,
	.reset		= dm9620_link_reset,
	.unbind     = dm9620_unbind,
};

   
#if 0   
//VIDs  //[PID]
// [..] [9621]=
	0x0000
	 ...
	0x0a46
	{32 Items}
//VIDs  //[PID]
// [..] [1269]=
	0x0000
	 ...
	0x0a46
	{32 Items}

//[VID] PIDs, PIDs
//[0000] [..]=
   [9621]* [1269]*
	9620    1268
	9601    1261
	9600    1260
	9421	1249
	9420	1248
	9401	1241
	9400	1240
   (9221)  (1229)
	9220 	1228
	9201
	9200
	9021	1209
	9020	1208
	9001
	9000
   (8621)  (1069)
	8620    1068
	8601	1061
	8600	1060
	8421	1049
	8420	1048	
	8401	1041
	8400	1040
   (8221)  (1029)
	8220 	1028
	8201	
	8200
	8021	1009
	8020	1008
	8001
	8000
	
   (1621)  (0269)
	1620 	0268
	1601	0261
	1600	0260
	1421	0249
	1420	0248
	1401	0241
	1400	0240
   (1221)  (0229)
	1220 	0228
	1201	
	1200	
	1021	0209
	1020	0208
	1001	
	1000	
   (0621)  (0069)
	0620 	0068
	0601	0061
	0600	0060
	0421	0049
	0420	0048
	0401	0041
	0400	0040
   (0221)  (0029)
	0220 	0028
	0201	
	0200	
	0021	0009
	0020	0008
	0001	
	0000	
{64 Items}{48 Items}
#endif

static const struct usb_device_id products[] = {
  	{USB_DEVICE(0x0a46, 0x9622), .driver_info = (unsigned long)&dm9620_info,}, /* Davicom 9622 */
	         {USB_DEVICE(0x0000, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9621), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9620), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9600), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9601), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9421), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9420), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9401), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9400), .driver_info = (unsigned long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x9000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x8000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x1000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0621), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0620), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0600), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0601), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0421), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0420), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0401), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0400), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0221), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0220), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0201), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0200), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0021), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0020), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0001), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0000, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0002, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0004, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0006, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0040, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0042, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0044, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0046, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0200, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0202, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0204, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0206, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0240, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0242, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0244, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0246, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0800, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0802, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0804, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0806, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0840, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0842, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0844, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0846, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a00, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a02, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a04, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a06, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a40, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a42, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a44, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},
         {USB_DEVICE(0x0a46, 0x0000), .driver_info = (unsigned  long)&dm9620_info,},

	{},			// END
};

MODULE_DEVICE_TABLE(usb, products);

static struct usb_driver dm9620_driver = {
	.name = "dm9620",
	.id_table = products,
	.probe = usbnet_probe,
	.disconnect = usbnet_disconnect,
	.suspend = usbnet_suspend,
	.resume = usbnet_resume,
};




static int __init dm9620_init(void)
{
	return usb_register(&dm9620_driver);
}

static void __exit dm9620_exit(void)
{
	usb_deregister(&dm9620_driver);
}

module_init(dm9620_init);
module_exit(dm9620_exit);

MODULE_AUTHOR("Peter Korsgaard <jacmet@sunsite.dk>");
MODULE_DESCRIPTION("Davicom DM9620 USB 2.0 ethernet devices");
MODULE_LICENSE("GPL");

