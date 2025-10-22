ipheth.c is based on Kernel 3.0.27 url is following:
https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/drivers/net/usb/ipheth.c?h=v3.0.27

compare withe the latest codes on Kernel 5.1, there are following patches:
(https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/log/drivers/net/usb/ipheth.c?h=linux-5.1.y)
2019-06-19	usbnet: ipheth: fix racing condition
2018-11-27	usbnet: ipheth: fix potential recvmsg bug and recvmsg bug 2
2017-11-19	usbnet: ipheth: fix potential null pointer dereference in ipheth_carrier_set
2017-11-15	usbnet: ipheth: prevent TX queue timeouts when device not ready
2017-08-08	net: usb: ipheth: constify usb_device_id
2017-06-16	networking: introduce and use skb_put_data()
2014-06-03	Merge git://git.kernel.org/pub/scm/linux/kernel/git/davem/net
2014-06-02	ipheth: Add support for iPad 2 and iPad 3
2014-05-13	net: get rid of SET_ETHTOOL_OPS	Wilfried Klaebe
2014-01-16	drivers/net: delete non-required instances of include <linux/init.h>
2013-07-02	net: ipheth: Add USB ID for iPad mini
2012-10-18	usb/ipheth: Add iPhone 5 support
2012-06-25	ipheth: add support for iPad
2012-05-18	USB: Disable hub-initiated LPM for comms devices.
2012-04-25	USB: ipheth.c: remove err() usage

almost all patches has been used and run ok except following:
2018-11-27	usbnet: ipheth: fix potential recvmsg bug and recvmsg bug 2
->kernel 3.0.27 do not support dev_consume_skb_any, so use dev_kfree_skb_any instead.

2017-06-16	networking: introduce and use skb_put_data()
2014-01-16	drivers/net: delete non-required instances of include <linux/init.h>
2012-05-18	USB: Disable hub-initiated LPM for comms devices.
->kernel 3.0.27 do not support